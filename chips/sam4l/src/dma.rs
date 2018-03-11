//! Implementation of the PDCA DMA peripheral.

use core::{cmp, intrinsics};
use core::cell::Cell;
use core::ops::{BitAnd, BitOr, Not, Shr, Shl};
use kernel::common::regs::{IntLike, ReadOnly, ReadWrite, WriteOnly};
use kernel::common::VolatileCell;
use kernel::common::take_cell::TakeCell;
use pm;

/// Memory registers for a DMA channel. Section 16.6.1 of the datasheet.
#[repr(C)]
#[allow(dead_code)]
struct DMARegisters {
    mar: ReadWrite<u32, MemoryAddress::Register>,
    psr: ReadWrite<DMAPeripheral>,
    // psr: VolatileCell<DMAPeripheral>,
    _psr_padding: [u8; 3],
    tcr: ReadWrite<u32, TransferCounter::Register>,
    marr: ReadWrite<u32, MemoryAddressReload::Register>,
    tcrr: ReadWrite<u32, TransferCounter::Register>,
    cr: WriteOnly<u32, Control::Register>,
    mr: ReadWrite<u32, Mode::Register>,
    sr: ReadOnly<u32, Status::Register>,
    ier: WriteOnly<u32, Interrupt::Register>,
    idr: WriteOnly<u32, Interrupt::Register>,
    imr: ReadOnly<u32, Interrupt::Register>,
    isr: ReadOnly<u32, Interrupt::Register>,
}

register_bitfields![u32,
    MemoryAddress [
        MADDR OFFSET(0) NUMBITS(32) []
    ],

    MemoryAddressReload [
        MARV OFFSET(0) NUMBITS(32) []
    ],

    PeripheralSelect [
        /// Peripheral Identifier
        PID OFFSET(0) NUMBITS(8) []
    ],

    TransferCounter [
        /// Transfer Counter Value
        TCV OFFSET(0) NUMBITS(16) []
    ],

    Control [
        /// Transfer Error Clear
        ECLR 8,
        /// Transfer Disable
        TDIS 1,
        /// Transfer Enable
        TEN 0
    ],

    Mode [
        /// Ring Buffer
        RING OFFSET(3) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ],
        /// Event Trigger
        ETRIG OFFSET(2) NUMBITS(1) [
            StartOnRequest = 0,
            StartOnEvent = 1
        ],
        /// Size of Transfer
        SIZE OFFSET(0) NUMBITS(2) [
            Byte = 0,
            Halfword = 1,
            Word = 2
        ]
    ],

    Status [
        /// Transfer Enabled
        TEN 0
    ],

    Interrupt [
        /// Transfer Error
        TERR 2,
        /// Transfer Complete
        TRC 1,
        /// Reload Counter Zero
        RCZ 0
    ]
];

/// The PDCA's base addresses in memory (Section 7.1 of manual).
const DMA_BASE_ADDR: usize = 0x400A2000;

/// The number of bytes between each memory mapped DMA Channel (Section 16.6.1).
const DMA_CHANNEL_SIZE: usize = 0x40;

/// Shared counter that Keeps track of how many DMA channels are currently
/// active.
static mut NUM_ENABLED: usize = 0;

/// The DMA channel number. Each channel transfers data between memory and a
/// particular peripheral function (e.g., SPI read or SPI write, but not both
/// simultaneously). There are 16 available channels (Section 16.7).
#[derive(Copy, Clone)]
pub enum DMAChannelNum {
    // Relies on the fact that assigns values 0-15 to each constructor in order
    DMAChannel00 = 0,
    DMAChannel01 = 1,
    DMAChannel02 = 2,
    DMAChannel03 = 3,
    DMAChannel04 = 4,
    DMAChannel05 = 5,
    DMAChannel06 = 6,
    DMAChannel07 = 7,
    DMAChannel08 = 8,
    DMAChannel09 = 9,
    DMAChannel10 = 10,
    DMAChannel11 = 11,
    DMAChannel12 = 12,
    DMAChannel13 = 13,
    DMAChannel14 = 14,
    DMAChannel15 = 15,
}

/// The peripheral function a channel is assigned to (Section 16.7). `*_RX`
/// means transfer data from peripheral to memory, `*_TX` means transfer data
/// from memory to peripheral.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum DMAPeripheral {
    USART0_RX = 0,
    USART1_RX = 1,
    USART2_RX = 2,
    USART3_RX = 3,
    SPI_RX = 4,
    TWIM0_RX = 5,
    TWIM1_RX = 6,
    TWIM2_RX = 7,
    TWIM3_RX = 8,
    TWIS0_RX = 9,
    TWIS1_RX = 10,
    ADCIFE_RX = 11,
    CATB_RX = 12,
    IISC_CH0_RX = 14,
    IISC_CH1_RX = 15,
    PARC_RX = 16,
    AESA_RX = 17,
    USART0_TX = 18,
    USART1_TX = 19,
    USART2_TX = 20,
    USART3_TX = 21,
    SPI_TX = 22,
    TWIM0_TX = 23,
    TWIM1_TX = 24,
    TWIM2_TX = 25,
    TWIM3_TX = 26,
    TWIS0_TX = 27,
    TWIS1_TX = 28,
    ADCIFE_TX = 29,
    CATB_TX = 30,
    ABDACB_SDR0_TX = 31,
    ABDACB_SDR1_TX = 32,
    IISC_CH0_TX = 33,
    IISC_CH1_TX = 34,
    DACC_TX = 35,
    AESA_TX = 36,
    LCDCA_ACMDR_TX = 37,
    LCDCA_ABMDR_TX = 38,
    UNUSED39 = 39,
    UNUSED40 = 40,
    UNUSED41 = 41,
    UNUSED42 = 42,
    UNUSED43 = 43,
    UNUSED44 = 44,
    UNUSED45 = 45,
    UNUSED46 = 46,
    UNUSED47 = 47,
    UNUSED48 = 48,
    UNUSED49 = 49,
    UNUSED50 = 50,
    UNUSED51 = 51,
    UNUSED52 = 52,
    UNUSED53 = 53,
    UNUSED54 = 54,
    UNUSED55 = 55,
    UNUSED56 = 56,
    UNUSED57 = 57,
    UNUSED58 = 58,
    UNUSED59 = 59,
    UNUSED60 = 60,
    UNUSED61 = 61,
    UNUSED62 = 62,
    UNUSED63 = 63,
    UNUSED64 = 64,
    UNUSED65 = 65,
    UNUSED66 = 66,
    UNUSED67 = 67,
    UNUSED68 = 68,
    UNUSED69 = 69,
    UNUSED70 = 70,
    UNUSED71 = 71,
    UNUSED72 = 72,
    UNUSED73 = 73,
    UNUSED74 = 74,
    UNUSED75 = 75,
    UNUSED76 = 76,
    UNUSED77 = 77,
    UNUSED78 = 78,
    UNUSED79 = 79,
    UNUSED80 = 80,
    UNUSED81 = 81,
    UNUSED82 = 82,
    UNUSED83 = 83,
    UNUSED84 = 84,
    UNUSED85 = 85,
    UNUSED86 = 86,
    UNUSED87 = 87,
    UNUSED88 = 88,
    UNUSED89 = 89,
    UNUSED90 = 90,
    UNUSED91 = 91,
    UNUSED92 = 92,
    UNUSED93 = 93,
    UNUSED94 = 94,
    UNUSED95 = 95,
    UNUSED96 = 96,
    UNUSED97 = 97,
    UNUSED98 = 98,
    UNUSED99 = 99,
    UNUSED100 = 100,
    UNUSED101 = 101,
    UNUSED102 = 102,
    UNUSED103 = 103,
    UNUSED104 = 104,
    UNUSED105 = 105,
    UNUSED106 = 106,
    UNUSED107 = 107,
    UNUSED108 = 108,
    UNUSED109 = 109,
    UNUSED110 = 110,
    UNUSED111 = 111,
    UNUSED112 = 112,
    UNUSED113 = 113,
    UNUSED114 = 114,
    UNUSED115 = 115,
    UNUSED116 = 116,
    UNUSED117 = 117,
    UNUSED118 = 118,
    UNUSED119 = 119,
    UNUSED120 = 120,
    UNUSED121 = 121,
    UNUSED122 = 122,
    UNUSED123 = 123,
    UNUSED124 = 124,
    UNUSED125 = 125,
    UNUSED126 = 126,
    UNUSED127 = 127,
    UNUSED128 = 128,
    UNUSED129 = 129,
    UNUSED130 = 130,
    UNUSED131 = 131,
    UNUSED132 = 132,
    UNUSED133 = 133,
    UNUSED134 = 134,
    UNUSED135 = 135,
    UNUSED136 = 136,
    UNUSED137 = 137,
    UNUSED138 = 138,
    UNUSED139 = 139,
    UNUSED140 = 140,
    UNUSED141 = 141,
    UNUSED142 = 142,
    UNUSED143 = 143,
    UNUSED144 = 144,
    UNUSED145 = 145,
    UNUSED146 = 146,
    UNUSED147 = 147,
    UNUSED148 = 148,
    UNUSED149 = 149,
    UNUSED150 = 150,
    UNUSED151 = 151,
    UNUSED152 = 152,
    UNUSED153 = 153,
    UNUSED154 = 154,
    UNUSED155 = 155,
    UNUSED156 = 156,
    UNUSED157 = 157,
    UNUSED158 = 158,
    UNUSED159 = 159,
    UNUSED160 = 160,
    UNUSED161 = 161,
    UNUSED162 = 162,
    UNUSED163 = 163,
    UNUSED164 = 164,
    UNUSED165 = 165,
    UNUSED166 = 166,
    UNUSED167 = 167,
    UNUSED168 = 168,
    UNUSED169 = 169,
    UNUSED170 = 170,
    UNUSED171 = 171,
    UNUSED172 = 172,
    UNUSED173 = 173,
    UNUSED174 = 174,
    UNUSED175 = 175,
    UNUSED176 = 176,
    UNUSED177 = 177,
    UNUSED178 = 178,
    UNUSED179 = 179,
    UNUSED180 = 180,
    UNUSED181 = 181,
    UNUSED182 = 182,
    UNUSED183 = 183,
    UNUSED184 = 184,
    UNUSED185 = 185,
    UNUSED186 = 186,
    UNUSED187 = 187,
    UNUSED188 = 188,
    UNUSED189 = 189,
    UNUSED190 = 190,
    UNUSED191 = 191,
    UNUSED192 = 192,
    UNUSED193 = 193,
    UNUSED194 = 194,
    UNUSED195 = 195,
    UNUSED196 = 196,
    UNUSED197 = 197,
    UNUSED198 = 198,
    UNUSED199 = 199,
    UNUSED200 = 200,
    UNUSED201 = 201,
    UNUSED202 = 202,
    UNUSED203 = 203,
    UNUSED204 = 204,
    UNUSED205 = 205,
    UNUSED206 = 206,
    UNUSED207 = 207,
    UNUSED208 = 208,
    UNUSED209 = 209,
    UNUSED210 = 210,
    UNUSED211 = 211,
    UNUSED212 = 212,
    UNUSED213 = 213,
    UNUSED214 = 214,
    UNUSED215 = 215,
    UNUSED216 = 216,
    UNUSED217 = 217,
    UNUSED218 = 218,
    UNUSED219 = 219,
    UNUSED220 = 220,
    UNUSED221 = 221,
    UNUSED222 = 222,
    UNUSED223 = 223,
    UNUSED224 = 224,
    UNUSED225 = 225,
    UNUSED226 = 226,
    UNUSED227 = 227,
    UNUSED228 = 228,
    UNUSED229 = 229,
    UNUSED230 = 230,
    UNUSED231 = 231,
    UNUSED232 = 232,
    UNUSED233 = 233,
    UNUSED234 = 234,
    UNUSED235 = 235,
    UNUSED236 = 236,
    UNUSED237 = 237,
    UNUSED238 = 238,
    UNUSED239 = 239,
    UNUSED240 = 240,
    UNUSED241 = 241,
    UNUSED242 = 242,
    UNUSED243 = 243,
    UNUSED244 = 244,
    UNUSED245 = 245,
    UNUSED246 = 246,
    UNUSED247 = 247,
    UNUSED248 = 248,
    UNUSED249 = 249,
    UNUSED250 = 250,
    UNUSED251 = 251,
    UNUSED252 = 252,
    UNUSED253 = 253,
    UNUSED254 = 254,
    UNUSED255 = 255,
}

impl DMAPeripheral {
    fn lookup(self, num: u8) -> DMAPeripheral {
        match num {
            0 => DMAPeripheral::USART0_RX,
            1 => DMAPeripheral::USART1_RX,
            2 => DMAPeripheral::USART2_RX,
            3 => DMAPeripheral::USART3_RX,
            4 => DMAPeripheral::SPI_RX,
            5 => DMAPeripheral::TWIM0_RX,
            6 => DMAPeripheral::TWIM1_RX,
            7 => DMAPeripheral::TWIM2_RX,
            8 => DMAPeripheral::TWIM3_RX,
            9 => DMAPeripheral::TWIS0_RX,
            10 => DMAPeripheral::TWIS1_RX,
            11 => DMAPeripheral::ADCIFE_RX,
            12 => DMAPeripheral::CATB_RX,
            14 => DMAPeripheral::IISC_CH0_RX,
            15 => DMAPeripheral::IISC_CH1_RX,
            16 => DMAPeripheral::PARC_RX,
            17 => DMAPeripheral::AESA_RX,
            18 => DMAPeripheral::USART0_TX,
            19 => DMAPeripheral::USART1_TX,
            20 => DMAPeripheral::USART2_TX,
            21 => DMAPeripheral::USART3_TX,
            22 => DMAPeripheral::SPI_TX,
            23 => DMAPeripheral::TWIM0_TX,
            24 => DMAPeripheral::TWIM1_TX,
            25 => DMAPeripheral::TWIM2_TX,
            26 => DMAPeripheral::TWIM3_TX,
            27 => DMAPeripheral::TWIS0_TX,
            28 => DMAPeripheral::TWIS1_TX,
            29 => DMAPeripheral::ADCIFE_TX,
            30 => DMAPeripheral::CATB_TX,
            31 => DMAPeripheral::ABDACB_SDR0_TX,
            32 => DMAPeripheral::ABDACB_SDR1_TX,
            33 => DMAPeripheral::IISC_CH0_TX,
            34 => DMAPeripheral::IISC_CH1_TX,
            35 => DMAPeripheral::DACC_TX,
            36 => DMAPeripheral::AESA_TX,
            37 => DMAPeripheral::LCDCA_ACMDR_TX,
            38 => DMAPeripheral::LCDCA_ABMDR_TX,
            39 => DMAPeripheral::UNUSED39,
            40 => DMAPeripheral::UNUSED40,
            41 => DMAPeripheral::UNUSED41,
            42 => DMAPeripheral::UNUSED42,
            43 => DMAPeripheral::UNUSED43,
            44 => DMAPeripheral::UNUSED44,
            45 => DMAPeripheral::UNUSED45,
            46 => DMAPeripheral::UNUSED46,
            47 => DMAPeripheral::UNUSED47,
            48 => DMAPeripheral::UNUSED48,
            49 => DMAPeripheral::UNUSED49,
            50 => DMAPeripheral::UNUSED50,
            51 => DMAPeripheral::UNUSED51,
            52 => DMAPeripheral::UNUSED52,
            53 => DMAPeripheral::UNUSED53,
            54 => DMAPeripheral::UNUSED54,
            55 => DMAPeripheral::UNUSED55,
            56 => DMAPeripheral::UNUSED56,
            57 => DMAPeripheral::UNUSED57,
            58 => DMAPeripheral::UNUSED58,
            59 => DMAPeripheral::UNUSED59,
            60 => DMAPeripheral::UNUSED60,
            61 => DMAPeripheral::UNUSED61,
            62 => DMAPeripheral::UNUSED62,
            63 => DMAPeripheral::UNUSED63,
            64 => DMAPeripheral::UNUSED64,
            65 => DMAPeripheral::UNUSED65,
            66 => DMAPeripheral::UNUSED66,
            67 => DMAPeripheral::UNUSED67,
            68 => DMAPeripheral::UNUSED68,
            69 => DMAPeripheral::UNUSED69,
            70 => DMAPeripheral::UNUSED70,
            71 => DMAPeripheral::UNUSED71,
            72 => DMAPeripheral::UNUSED72,
            73 => DMAPeripheral::UNUSED73,
            74 => DMAPeripheral::UNUSED74,
            75 => DMAPeripheral::UNUSED75,
            76 => DMAPeripheral::UNUSED76,
            77 => DMAPeripheral::UNUSED77,
            78 => DMAPeripheral::UNUSED78,
            79 => DMAPeripheral::UNUSED79,
            80 => DMAPeripheral::UNUSED80,
            81 => DMAPeripheral::UNUSED81,
            82 => DMAPeripheral::UNUSED82,
            83 => DMAPeripheral::UNUSED83,
            84 => DMAPeripheral::UNUSED84,
            85 => DMAPeripheral::UNUSED85,
            86 => DMAPeripheral::UNUSED86,
            87 => DMAPeripheral::UNUSED87,
            88 => DMAPeripheral::UNUSED88,
            89 => DMAPeripheral::UNUSED89,
            90 => DMAPeripheral::UNUSED90,
            91 => DMAPeripheral::UNUSED91,
            92 => DMAPeripheral::UNUSED92,
            93 => DMAPeripheral::UNUSED93,
            94 => DMAPeripheral::UNUSED94,
            95 => DMAPeripheral::UNUSED95,
            96 => DMAPeripheral::UNUSED96,
            97 => DMAPeripheral::UNUSED97,
            98 => DMAPeripheral::UNUSED98,
            99 => DMAPeripheral::UNUSED99,
            100 => DMAPeripheral::UNUSED100,
            101 => DMAPeripheral::UNUSED101,
            102 => DMAPeripheral::UNUSED102,
            103 => DMAPeripheral::UNUSED103,
            104 => DMAPeripheral::UNUSED104,
            105 => DMAPeripheral::UNUSED105,
            106 => DMAPeripheral::UNUSED106,
            107 => DMAPeripheral::UNUSED107,
            108 => DMAPeripheral::UNUSED108,
            109 => DMAPeripheral::UNUSED109,
            110 => DMAPeripheral::UNUSED110,
            111 => DMAPeripheral::UNUSED111,
            112 => DMAPeripheral::UNUSED112,
            113 => DMAPeripheral::UNUSED113,
            114 => DMAPeripheral::UNUSED114,
            115 => DMAPeripheral::UNUSED115,
            116 => DMAPeripheral::UNUSED116,
            117 => DMAPeripheral::UNUSED117,
            118 => DMAPeripheral::UNUSED118,
            119 => DMAPeripheral::UNUSED119,
            120 => DMAPeripheral::UNUSED120,
            121 => DMAPeripheral::UNUSED121,
            122 => DMAPeripheral::UNUSED122,
            123 => DMAPeripheral::UNUSED123,
            124 => DMAPeripheral::UNUSED124,
            125 => DMAPeripheral::UNUSED125,
            126 => DMAPeripheral::UNUSED126,
            127 => DMAPeripheral::UNUSED127,
            128 => DMAPeripheral::UNUSED128,
            129 => DMAPeripheral::UNUSED129,
            130 => DMAPeripheral::UNUSED130,
            131 => DMAPeripheral::UNUSED131,
            132 => DMAPeripheral::UNUSED132,
            133 => DMAPeripheral::UNUSED133,
            134 => DMAPeripheral::UNUSED134,
            135 => DMAPeripheral::UNUSED135,
            136 => DMAPeripheral::UNUSED136,
            137 => DMAPeripheral::UNUSED137,
            138 => DMAPeripheral::UNUSED138,
            139 => DMAPeripheral::UNUSED139,
            140 => DMAPeripheral::UNUSED140,
            141 => DMAPeripheral::UNUSED141,
            142 => DMAPeripheral::UNUSED142,
            143 => DMAPeripheral::UNUSED143,
            144 => DMAPeripheral::UNUSED144,
            145 => DMAPeripheral::UNUSED145,
            146 => DMAPeripheral::UNUSED146,
            147 => DMAPeripheral::UNUSED147,
            148 => DMAPeripheral::UNUSED148,
            149 => DMAPeripheral::UNUSED149,
            150 => DMAPeripheral::UNUSED150,
            151 => DMAPeripheral::UNUSED151,
            152 => DMAPeripheral::UNUSED152,
            153 => DMAPeripheral::UNUSED153,
            154 => DMAPeripheral::UNUSED154,
            155 => DMAPeripheral::UNUSED155,
            156 => DMAPeripheral::UNUSED156,
            157 => DMAPeripheral::UNUSED157,
            158 => DMAPeripheral::UNUSED158,
            159 => DMAPeripheral::UNUSED159,
            160 => DMAPeripheral::UNUSED160,
            161 => DMAPeripheral::UNUSED161,
            162 => DMAPeripheral::UNUSED162,
            163 => DMAPeripheral::UNUSED163,
            164 => DMAPeripheral::UNUSED164,
            165 => DMAPeripheral::UNUSED165,
            166 => DMAPeripheral::UNUSED166,
            167 => DMAPeripheral::UNUSED167,
            168 => DMAPeripheral::UNUSED168,
            169 => DMAPeripheral::UNUSED169,
            170 => DMAPeripheral::UNUSED170,
            171 => DMAPeripheral::UNUSED171,
            172 => DMAPeripheral::UNUSED172,
            173 => DMAPeripheral::UNUSED173,
            174 => DMAPeripheral::UNUSED174,
            175 => DMAPeripheral::UNUSED175,
            176 => DMAPeripheral::UNUSED176,
            177 => DMAPeripheral::UNUSED177,
            178 => DMAPeripheral::UNUSED178,
            179 => DMAPeripheral::UNUSED179,
            180 => DMAPeripheral::UNUSED180,
            181 => DMAPeripheral::UNUSED181,
            182 => DMAPeripheral::UNUSED182,
            183 => DMAPeripheral::UNUSED183,
            184 => DMAPeripheral::UNUSED184,
            185 => DMAPeripheral::UNUSED185,
            186 => DMAPeripheral::UNUSED186,
            187 => DMAPeripheral::UNUSED187,
            188 => DMAPeripheral::UNUSED188,
            189 => DMAPeripheral::UNUSED189,
            190 => DMAPeripheral::UNUSED190,
            191 => DMAPeripheral::UNUSED191,
            192 => DMAPeripheral::UNUSED192,
            193 => DMAPeripheral::UNUSED193,
            194 => DMAPeripheral::UNUSED194,
            195 => DMAPeripheral::UNUSED195,
            196 => DMAPeripheral::UNUSED196,
            197 => DMAPeripheral::UNUSED197,
            198 => DMAPeripheral::UNUSED198,
            199 => DMAPeripheral::UNUSED199,
            200 => DMAPeripheral::UNUSED200,
            201 => DMAPeripheral::UNUSED201,
            202 => DMAPeripheral::UNUSED202,
            203 => DMAPeripheral::UNUSED203,
            204 => DMAPeripheral::UNUSED204,
            205 => DMAPeripheral::UNUSED205,
            206 => DMAPeripheral::UNUSED206,
            207 => DMAPeripheral::UNUSED207,
            208 => DMAPeripheral::UNUSED208,
            209 => DMAPeripheral::UNUSED209,
            210 => DMAPeripheral::UNUSED210,
            211 => DMAPeripheral::UNUSED211,
            212 => DMAPeripheral::UNUSED212,
            213 => DMAPeripheral::UNUSED213,
            214 => DMAPeripheral::UNUSED214,
            215 => DMAPeripheral::UNUSED215,
            216 => DMAPeripheral::UNUSED216,
            217 => DMAPeripheral::UNUSED217,
            218 => DMAPeripheral::UNUSED218,
            219 => DMAPeripheral::UNUSED219,
            220 => DMAPeripheral::UNUSED220,
            221 => DMAPeripheral::UNUSED221,
            222 => DMAPeripheral::UNUSED222,
            223 => DMAPeripheral::UNUSED223,
            224 => DMAPeripheral::UNUSED224,
            225 => DMAPeripheral::UNUSED225,
            226 => DMAPeripheral::UNUSED226,
            227 => DMAPeripheral::UNUSED227,
            228 => DMAPeripheral::UNUSED228,
            229 => DMAPeripheral::UNUSED229,
            230 => DMAPeripheral::UNUSED230,
            231 => DMAPeripheral::UNUSED231,
            232 => DMAPeripheral::UNUSED232,
            233 => DMAPeripheral::UNUSED233,
            234 => DMAPeripheral::UNUSED234,
            235 => DMAPeripheral::UNUSED235,
            236 => DMAPeripheral::UNUSED236,
            237 => DMAPeripheral::UNUSED237,
            238 => DMAPeripheral::UNUSED238,
            239 => DMAPeripheral::UNUSED239,
            240 => DMAPeripheral::UNUSED240,
            241 => DMAPeripheral::UNUSED241,
            242 => DMAPeripheral::UNUSED242,
            243 => DMAPeripheral::UNUSED243,
            244 => DMAPeripheral::UNUSED244,
            245 => DMAPeripheral::UNUSED245,
            246 => DMAPeripheral::UNUSED246,
            247 => DMAPeripheral::UNUSED247,
            248 => DMAPeripheral::UNUSED248,
            249 => DMAPeripheral::UNUSED249,
            250 => DMAPeripheral::UNUSED250,
            251 => DMAPeripheral::UNUSED251,
            252 => DMAPeripheral::UNUSED252,
            253 => DMAPeripheral::UNUSED253,
            254 => DMAPeripheral::UNUSED254,
            255 => DMAPeripheral::UNUSED255,
            _ => DMAPeripheral::UNUSED255,
        }
    }
}

impl IntLike for DMAPeripheral {
    fn zero() -> Self {
        DMAPeripheral::USART0_RX
    }
}

impl BitAnd for DMAPeripheral {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self {
        self.lookup((self as u8) & (rhs as u8))
    }
}

impl BitOr for DMAPeripheral {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self {
        self.lookup((self as u8) | (rhs as u8))
    }
}

impl Not for DMAPeripheral {
    type Output = Self;

    fn not(self) -> Self {
        self.lookup(!(self as u8))
    }
}

impl Shr<u32> for DMAPeripheral {
    type Output = Self;

    fn shr(self, rhs: u32) -> Self {
        self.lookup((self as u8) >> rhs)
    }
}

impl Shl<u32> for DMAPeripheral {
    type Output = Self;

    fn shl(self, rhs: u32) -> Self {
        self.lookup((self as u8) << rhs)
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum DMAWidth {
    Width8Bit = 0,
    Width16Bit = 1,
    Width32Bit = 2,
}

pub static mut DMA_CHANNELS: [DMAChannel; 16] = [
    DMAChannel::new(DMAChannelNum::DMAChannel00),
    DMAChannel::new(DMAChannelNum::DMAChannel01),
    DMAChannel::new(DMAChannelNum::DMAChannel02),
    DMAChannel::new(DMAChannelNum::DMAChannel03),
    DMAChannel::new(DMAChannelNum::DMAChannel04),
    DMAChannel::new(DMAChannelNum::DMAChannel05),
    DMAChannel::new(DMAChannelNum::DMAChannel06),
    DMAChannel::new(DMAChannelNum::DMAChannel07),
    DMAChannel::new(DMAChannelNum::DMAChannel08),
    DMAChannel::new(DMAChannelNum::DMAChannel09),
    DMAChannel::new(DMAChannelNum::DMAChannel10),
    DMAChannel::new(DMAChannelNum::DMAChannel11),
    DMAChannel::new(DMAChannelNum::DMAChannel12),
    DMAChannel::new(DMAChannelNum::DMAChannel13),
    DMAChannel::new(DMAChannelNum::DMAChannel14),
    DMAChannel::new(DMAChannelNum::DMAChannel15),
];

pub struct DMAChannel {
    registers: *mut DMARegisters,
    client: Cell<Option<&'static DMAClient>>,
    width: Cell<DMAWidth>,
    enabled: Cell<bool>,
    buffer: TakeCell<'static, [u8]>,
}

pub trait DMAClient {
    fn xfer_done(&self, pid: DMAPeripheral);
}

impl DMAChannel {
    const fn new(channel: DMAChannelNum) -> DMAChannel {
        DMAChannel {
            registers: (DMA_BASE_ADDR + (channel as usize) * DMA_CHANNEL_SIZE) as *mut DMARegisters,
            client: Cell::new(None),
            width: Cell::new(DMAWidth::Width8Bit),
            enabled: Cell::new(false),
            buffer: TakeCell::empty(),
        }
    }

    pub fn initialize(&self, client: &'static mut DMAClient, width: DMAWidth) {
        self.client.set(Some(client));
        self.width.set(width);
    }

    pub fn enable(&self) {
        unsafe {
            pm::enable_clock(pm::Clock::HSB(pm::HSBClock::PDCA));
            pm::enable_clock(pm::Clock::PBB(pm::PBBClock::PDCA));
        }
        if !self.enabled.get() {
            unsafe {
                let num_enabled = intrinsics::atomic_xadd(&mut NUM_ENABLED, 1);
                if num_enabled == 1 {
                    pm::enable_clock(pm::Clock::HSB(pm::HSBClock::PDCA));
                    pm::enable_clock(pm::Clock::PBB(pm::PBBClock::PDCA));
                }
            }
            let registers: &DMARegisters = unsafe { &*self.registers };
            // Disable all interrupts
            registers.idr.write(Interrupt::TERR::SET + Interrupt::TRC::SET + Interrupt::RCZ::SET);

            self.enabled.set(true);
        }
    }

    pub fn disable(&self) {
        if self.enabled.get() {
            unsafe {
                let num_enabled = intrinsics::atomic_xsub(&mut NUM_ENABLED, 1);
                if num_enabled == 1 {
                    pm::disable_clock(pm::Clock::HSB(pm::HSBClock::PDCA));
                    pm::disable_clock(pm::Clock::PBB(pm::PBBClock::PDCA));
                }
            }
            let registers: &DMARegisters = unsafe { &*self.registers };
            registers.cr.write(Control::TDIS::SET);
            self.enabled.set(false);
        }
    }

    pub fn handle_interrupt(&mut self) {
        let registers: &DMARegisters = unsafe { &*self.registers };
        registers.idr.write(Interrupt::TERR::SET + Interrupt::TRC::SET + Interrupt::RCZ::SET);
        // let channel = registers.psr.read(PeripheralSelect::PID);
        let channel = registers.psr.get();

        self.client.get().as_mut().map(|client| {
            client.xfer_done(channel);
        });
    }

    pub fn start_xfer(&self) {
        let registers: &DMARegisters = unsafe { &*self.registers };
        registers.cr.write(Control::TEN::SET);
    }

    pub fn prepare_xfer(&self, pid: DMAPeripheral, buf: &'static mut [u8], mut len: usize) {
        // TODO(alevy): take care of zero length case

        let registers: &DMARegisters = unsafe { &*self.registers };

        let maxlen = buf.len() / match self.width.get() {
                DMAWidth::Width8Bit /*  DMA is acting on bytes     */ => 1,
                DMAWidth::Width16Bit /* DMA is acting on halfwords */ => 2,
                DMAWidth::Width32Bit /* DMA is acting on words     */ => 4,
            };
        len = cmp::min(len, maxlen);
        registers.mr.write(Mode::SIZE.val(self.width.get() as u32));

        // registers.psr.write(PeripheralSelect::PID.val(pid));
        registers.psr.set(pid);
        registers.marr.write(MemoryAddressReload::MARV.val(&buf[0] as *const u8 as u32));
        registers.tcrr.write(TransferCounter::TCV.val(len as u32));

        registers.ier.write(Interrupt::TRC::SET);

        // Store the buffer reference in the TakeCell so it can be returned to
        // the caller in `handle_interrupt`
        self.buffer.replace(buf);
    }

    pub fn do_xfer(&self, pid: DMAPeripheral, buf: &'static mut [u8], len: usize) {
        self.prepare_xfer(pid, buf, len);
        self.start_xfer();
    }

    /// Aborts any current transactions and returns the buffer used in the
    /// transaction.
    pub fn abort_xfer(&self) -> Option<&'static mut [u8]> {
        let registers: &DMARegisters = unsafe { &*self.registers };
        registers.idr.write(Interrupt::TERR::SET + Interrupt::TRC::SET + Interrupt::RCZ::SET);

        // Reset counter
        registers.tcr.write(TransferCounter::TCV.val(0));

        self.buffer.take()
    }

    pub fn transfer_counter(&self) -> usize {
        let registers: &DMARegisters = unsafe { &*self.registers };
        registers.tcr.read(TransferCounter::TCV) as usize
    }
}
