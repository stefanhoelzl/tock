//! Implements the Tock bootloader.

use core::cell::Cell;
use core::cmp;
use kernel::{AppId, AppSlice, Container, Callback, Shared, Driver, ReturnCode};
use kernel::common::take_cell::TakeCell;
use kernel::hil;
use kernel::process::Error;


pub static mut BUF: [u8; 512] = [0; 512];


const ESCAPE_CHAR: u8 = 0xFC;
const CMD_PING: u8 = 0x01;

const RES_PONG: u8 = 0x11;

pub struct Bootloader<'a, U: hil::uart::UARTAdvanced + 'a, F: hil::flash::Flash + 'a, G: hil::gpio::Pin + 'a> {
    uart: &'a U,
    flash: &'a F,
    select_pin: &'a G,
    /// Buffer correctly sized for the underlying flash page size.
    page_buffer: TakeCell<'a, F::Page>,
    // in_progress: Cell<Option<AppId>>,
    buffer: TakeCell<'static, [u8]>,
    // baud_rate: u32,
}

impl<'a, U: hil::uart::UARTAdvanced + 'a, F: hil::flash::Flash + 'a, G: hil::gpio::Pin + 'a> Bootloader<'a, U, F, G> {
    pub fn new(uart: &'a U, flash: &'a F, select_pin: &'a G,
               page_buffer: &'static mut F::Page,
               buffer: &'static mut [u8])
               -> Bootloader<'a, U, F, G> {
        Bootloader {
            uart: uart,
            flash: flash,
            select_pin: select_pin,
            // in_progress: Cell::new(None),
            page_buffer: TakeCell::new(page_buffer),
            buffer: TakeCell::new(buffer),
            // baud_rate: baud_rate,
        }
    }

    pub fn initialize(&self) {
        self.select_pin.make_input();

        // Check the select pin to see if we should enter bootloader mode.
        let mut samples = 10000;
        let mut active = 0;
        let mut inactive = 0;
        while samples > 0 {
            if self.select_pin.read() == false {
                active += 1;
            } else {
                inactive += 1;
            }
            samples -= 1;
        }

        if active > inactive {
            // Looks like we do want bootloader mode.

            // Setup UART and start listening.
            self.uart.init(hil::uart::UARTParams {
                baud_rate: 115200,
                stop_bits: hil::uart::StopBits::One,
                parity: hil::uart::Parity::None,
                hw_flow_control: false,
            });

            self.buffer.take().map(|buffer| self.uart.receive_automatic(buffer, 250));


        } else {
            // Jump to the kernel and start the real code.
        }


    }

    // /// Internal helper function for setting up a new send transaction
    // fn send_new(&self, app_id: AppId, app: &mut App, len: usize) -> ReturnCode {
    //     match app.write_buffer.take() {
    //         Some(slice) => {
    //             app.write_len = cmp::min(len, slice.len());
    //             app.write_remaining = app.write_len;
    //             self.send(app_id, app, slice);
    //             ReturnCode::SUCCESS
    //         }
    //         None => ReturnCode::EBUSY,
    //     }
    // }

    // /// Internal helper function for continuing a previously set up transaction
    // /// Returns true if this send is still active, or false if it has completed
    // fn send_continue(&self, app_id: AppId, app: &mut App) -> Result<bool, ReturnCode> {
    //     if app.write_remaining > 0 {
    //         app.write_buffer.take().map_or(Err(ReturnCode::ERESERVE), |slice| {
    //             self.send(app_id, app, slice);
    //             Ok(true)
    //         })
    //     } else {
    //         Ok(false)
    //     }
    // }

    // /// Internal helper function for sending data for an existing transaction.
    // /// Cannot fail. If can't send now, it will schedule for sending later.
    // fn send(&self, app_id: AppId, app: &mut App, slice: AppSlice<Shared, u8>) {
    //     if self.in_progress.get().is_none() {
    //         self.in_progress.set(Some(app_id));
    //         self.tx_buffer.take().map(|buffer| {
    //             let mut transaction_len = app.write_remaining;
    //             for (i, c) in slice.as_ref()[slice.len() - app.write_remaining..slice.len()]
    //                 .iter()
    //                 .enumerate() {
    //                 if buffer.len() <= i {
    //                     break;
    //                 }
    //                 buffer[i] = *c;
    //             }

    //             // Check if everything we wanted to print
    //             // fit in the buffer.
    //             if app.write_remaining > buffer.len() {
    //                 transaction_len = buffer.len();
    //                 app.write_remaining -= buffer.len();
    //                 app.write_buffer = Some(slice);
    //             } else {
    //                 app.write_remaining = 0;
    //             }

    //             self.uart.transmit(buffer, transaction_len);
    //         });
    //     } else {
    //         app.pending_write = true;
    //         app.write_buffer = Some(slice);
    //     }
    // }
}

impl<'a, U: hil::uart::UARTAdvanced + 'a, F: hil::flash::Flash + 'a, G: hil::gpio::Pin + 'a> hil::uart::Client for Bootloader<'a, U, F, G> {
    fn transmit_complete(&self, buffer: &'static mut [u8], _error: hil::uart::Error) {
        // // Either print more from the AppSlice or send a callback to the
        // // application.
        // self.tx_buffer.replace(buffer);
        // self.in_progress.get().map(|appid| {
        //     self.in_progress.set(None);
        //     self.apps.enter(appid, |app, _| {
        //         match self.send_continue(appid, app) {
        //             Ok(more_to_send) => {
        //                 if !more_to_send {
        //                     // Go ahead and signal the application
        //                     let written = app.write_len;
        //                     app.write_len = 0;
        //                     app.write_callback.map(|mut cb| { cb.schedule(written, 0, 0); });
        //                 }
        //             }
        //             Err(return_code) => {
        //                 // XXX This shouldn't ever happen?
        //                 app.write_len = 0;
        //                 app.write_remaining = 0;
        //                 app.pending_write = false;
        //                 let r0 = isize::from(return_code) as usize;
        //                 app.write_callback.map(|mut cb| { cb.schedule(r0, 0, 0); });
        //             }
        //         }
        //     })
        // });

        // // If we are not printing more from the current AppSlice,
        // // see if any other applications have pending messages.
        // if self.in_progress.get().is_none() {
        //     for cntr in self.apps.iter() {
        //         let started_tx = cntr.enter(|app, _| {
        //             if app.pending_write {
        //                 app.pending_write = false;
        //                 match self.send_continue(app.appid(), app) {
        //                     Ok(more_to_send) => more_to_send,
        //                     Err(return_code) => {
        //                         // XXX This shouldn't ever happen?
        //                         app.write_len = 0;
        //                         app.write_remaining = 0;
        //                         app.pending_write = false;
        //                         let r0 = isize::from(return_code) as usize;
        //                         app.write_callback.map(|mut cb| { cb.schedule(r0, 0, 0); });
        //                         false
        //                     }
        //                 }
        //             } else {
        //                 false
        //             }
        //         });
        //         if started_tx {
        //             break;
        //         }
        //     }
        // }

        self.buffer.replace(buffer);
    }

    fn receive_complete(&self,
                        buffer: &'static mut [u8],
                        rx_len: usize,
                        _error: hil::uart::Error) {


        if rx_len >= 2 {
            // Check for escape character then the command byte.

            if buffer[rx_len-2] == ESCAPE_CHAR && buffer[rx_len-1] != ESCAPE_CHAR {
                // This looks like a valid command.

                match buffer[rx_len-1] {
                    CMD_PING => {
                        buffer[0] = ESCAPE_CHAR;
                        buffer[1] = RES_PONG;

                        self.uart.transmit(buffer, 2);
                    }

                    _ => {}
                }
            }
        }
    }
}

impl<'a, U: hil::uart::UARTAdvanced + 'a, F: hil::flash::Flash + 'a, G: hil::gpio::Pin + 'a> hil::flash::Client<F> for Bootloader<'a, U, F, G> {
    fn read_complete(&self, pagebuffer: &'static mut F::Page, _error: hil::flash::Error) {

    }

    fn write_complete(&self, pagebuffer: &'static mut F::Page, _error: hil::flash::Error) {

    }

    fn erase_complete(&self, _error: hil::flash::Error) {}
}
