#[derive(Copy, Clone)]
pub struct HuffmanEntry {
    pub code: u16,
    pub bits: u8,
}

pub const HUFFMAN_TABLE: [HuffmanEntry; 257] = [
    HuffmanEntry { bits: 1, code: 0 },
    HuffmanEntry { bits: 5, code: 7 },
    HuffmanEntry { bits: 4, code: 5 },
    HuffmanEntry { bits: 4, code: 13 },
    HuffmanEntry { bits: 7, code: 15 },
    HuffmanEntry { bits: 6, code: 23 },
    HuffmanEntry { bits: 8, code: 47 },
    HuffmanEntry { bits: 8, code: 175 },
    HuffmanEntry { bits: 8, code: 111 },
    HuffmanEntry { bits: 9, code: 95 },
    HuffmanEntry { bits: 9, code: 351 },
    HuffmanEntry { bits: 10, code: 63 },
    HuffmanEntry {
        bits: 10,
        code: 575,
    },
    HuffmanEntry {
        bits: 10,
        code: 319,
    },
    HuffmanEntry {
        bits: 11,
        code: 703,
    },
    HuffmanEntry {
        bits: 11,
        code: 1727,
    },
    HuffmanEntry {
        bits: 11,
        code: 447,
    },
    HuffmanEntry {
        bits: 12,
        code: 1151,
    },
    HuffmanEntry {
        bits: 14,
        code: 639,
    },
    HuffmanEntry {
        bits: 15,
        code: 4735,
    },
    HuffmanEntry {
        bits: 15,
        code: 21119,
    },
    HuffmanEntry {
        bits: 15,
        code: 12927,
    },
    HuffmanEntry {
        bits: 15,
        code: 29311,
    },
    HuffmanEntry {
        bits: 15,
        code: 2687,
    },
    HuffmanEntry {
        bits: 15,
        code: 19071,
    },
    HuffmanEntry {
        bits: 15,
        code: 10879,
    },
    HuffmanEntry {
        bits: 15,
        code: 27263,
    },
    HuffmanEntry {
        bits: 15,
        code: 6783,
    },
    HuffmanEntry {
        bits: 15,
        code: 23167,
    },
    HuffmanEntry {
        bits: 15,
        code: 14975,
    },
    HuffmanEntry {
        bits: 15,
        code: 31359,
    },
    HuffmanEntry {
        bits: 15,
        code: 1663,
    },
    HuffmanEntry {
        bits: 15,
        code: 18047,
    },
    HuffmanEntry {
        bits: 15,
        code: 9855,
    },
    HuffmanEntry {
        bits: 15,
        code: 26239,
    },
    HuffmanEntry {
        bits: 15,
        code: 5759,
    },
    HuffmanEntry {
        bits: 15,
        code: 22143,
    },
    HuffmanEntry {
        bits: 15,
        code: 13951,
    },
    HuffmanEntry {
        bits: 15,
        code: 30335,
    },
    HuffmanEntry {
        bits: 15,
        code: 3711,
    },
    HuffmanEntry {
        bits: 15,
        code: 20095,
    },
    HuffmanEntry {
        bits: 15,
        code: 11903,
    },
    HuffmanEntry {
        bits: 15,
        code: 28287,
    },
    HuffmanEntry {
        bits: 15,
        code: 7807,
    },
    HuffmanEntry {
        bits: 15,
        code: 24191,
    },
    HuffmanEntry {
        bits: 15,
        code: 15999,
    },
    HuffmanEntry {
        bits: 15,
        code: 32383,
    },
    HuffmanEntry {
        bits: 15,
        code: 383,
    },
    HuffmanEntry {
        bits: 15,
        code: 16767,
    },
    HuffmanEntry {
        bits: 15,
        code: 8575,
    },
    HuffmanEntry {
        bits: 15,
        code: 24959,
    },
    HuffmanEntry {
        bits: 15,
        code: 4479,
    },
    HuffmanEntry {
        bits: 15,
        code: 20863,
    },
    HuffmanEntry {
        bits: 15,
        code: 12671,
    },
    HuffmanEntry {
        bits: 15,
        code: 29055,
    },
    HuffmanEntry {
        bits: 15,
        code: 2431,
    },
    HuffmanEntry {
        bits: 15,
        code: 18815,
    },
    HuffmanEntry {
        bits: 15,
        code: 10623,
    },
    HuffmanEntry {
        bits: 15,
        code: 27007,
    },
    HuffmanEntry {
        bits: 15,
        code: 6527,
    },
    HuffmanEntry {
        bits: 15,
        code: 22911,
    },
    HuffmanEntry {
        bits: 15,
        code: 14719,
    },
    HuffmanEntry {
        bits: 15,
        code: 31103,
    },
    HuffmanEntry {
        bits: 15,
        code: 1407,
    },
    HuffmanEntry {
        bits: 15,
        code: 17791,
    },
    HuffmanEntry {
        bits: 15,
        code: 9599,
    },
    HuffmanEntry {
        bits: 15,
        code: 25983,
    },
    HuffmanEntry {
        bits: 15,
        code: 5503,
    },
    HuffmanEntry {
        bits: 15,
        code: 21887,
    },
    HuffmanEntry {
        bits: 15,
        code: 13695,
    },
    HuffmanEntry {
        bits: 15,
        code: 30079,
    },
    HuffmanEntry {
        bits: 15,
        code: 3455,
    },
    HuffmanEntry {
        bits: 15,
        code: 19839,
    },
    HuffmanEntry {
        bits: 15,
        code: 11647,
    },
    HuffmanEntry {
        bits: 15,
        code: 28031,
    },
    HuffmanEntry {
        bits: 15,
        code: 7551,
    },
    HuffmanEntry {
        bits: 15,
        code: 23935,
    },
    HuffmanEntry {
        bits: 15,
        code: 15743,
    },
    HuffmanEntry {
        bits: 15,
        code: 32127,
    },
    HuffmanEntry {
        bits: 15,
        code: 895,
    },
    HuffmanEntry {
        bits: 15,
        code: 17279,
    },
    HuffmanEntry {
        bits: 15,
        code: 9087,
    },
    HuffmanEntry {
        bits: 15,
        code: 25471,
    },
    HuffmanEntry {
        bits: 15,
        code: 4991,
    },
    HuffmanEntry {
        bits: 15,
        code: 21375,
    },
    HuffmanEntry {
        bits: 15,
        code: 13183,
    },
    HuffmanEntry {
        bits: 15,
        code: 29567,
    },
    HuffmanEntry {
        bits: 15,
        code: 2943,
    },
    HuffmanEntry {
        bits: 15,
        code: 19327,
    },
    HuffmanEntry {
        bits: 15,
        code: 11135,
    },
    HuffmanEntry {
        bits: 15,
        code: 27519,
    },
    HuffmanEntry {
        bits: 15,
        code: 7039,
    },
    HuffmanEntry {
        bits: 15,
        code: 23423,
    },
    HuffmanEntry {
        bits: 15,
        code: 15231,
    },
    HuffmanEntry {
        bits: 15,
        code: 31615,
    },
    HuffmanEntry {
        bits: 15,
        code: 1919,
    },
    HuffmanEntry {
        bits: 15,
        code: 18303,
    },
    HuffmanEntry {
        bits: 15,
        code: 10111,
    },
    HuffmanEntry {
        bits: 15,
        code: 26495,
    },
    HuffmanEntry {
        bits: 15,
        code: 6015,
    },
    HuffmanEntry {
        bits: 15,
        code: 22399,
    },
    HuffmanEntry {
        bits: 15,
        code: 14207,
    },
    HuffmanEntry {
        bits: 15,
        code: 30591,
    },
    HuffmanEntry {
        bits: 15,
        code: 3967,
    },
    HuffmanEntry {
        bits: 15,
        code: 20351,
    },
    HuffmanEntry {
        bits: 15,
        code: 12159,
    },
    HuffmanEntry {
        bits: 15,
        code: 28543,
    },
    HuffmanEntry {
        bits: 15,
        code: 8063,
    },
    HuffmanEntry {
        bits: 15,
        code: 24447,
    },
    HuffmanEntry {
        bits: 15,
        code: 16255,
    },
    HuffmanEntry {
        bits: 15,
        code: 32639,
    },
    HuffmanEntry {
        bits: 15,
        code: 255,
    },
    HuffmanEntry {
        bits: 15,
        code: 16639,
    },
    HuffmanEntry {
        bits: 15,
        code: 8447,
    },
    HuffmanEntry {
        bits: 15,
        code: 24831,
    },
    HuffmanEntry {
        bits: 15,
        code: 4351,
    },
    HuffmanEntry {
        bits: 15,
        code: 20735,
    },
    HuffmanEntry {
        bits: 15,
        code: 12543,
    },
    HuffmanEntry {
        bits: 15,
        code: 28927,
    },
    HuffmanEntry {
        bits: 15,
        code: 2303,
    },
    HuffmanEntry {
        bits: 15,
        code: 18687,
    },
    HuffmanEntry {
        bits: 15,
        code: 10495,
    },
    HuffmanEntry {
        bits: 15,
        code: 26879,
    },
    HuffmanEntry {
        bits: 15,
        code: 6399,
    },
    HuffmanEntry {
        bits: 15,
        code: 22783,
    },
    HuffmanEntry {
        bits: 15,
        code: 14591,
    },
    HuffmanEntry {
        bits: 15,
        code: 30975,
    },
    HuffmanEntry {
        bits: 15,
        code: 1279,
    },
    HuffmanEntry {
        bits: 15,
        code: 17663,
    },
    HuffmanEntry {
        bits: 15,
        code: 9471,
    },
    HuffmanEntry {
        bits: 15,
        code: 25855,
    },
    HuffmanEntry {
        bits: 15,
        code: 5375,
    },
    HuffmanEntry {
        bits: 15,
        code: 21759,
    },
    HuffmanEntry {
        bits: 15,
        code: 13567,
    },
    HuffmanEntry {
        bits: 15,
        code: 29951,
    },
    HuffmanEntry {
        bits: 15,
        code: 3327,
    },
    HuffmanEntry {
        bits: 15,
        code: 19711,
    },
    HuffmanEntry {
        bits: 15,
        code: 11519,
    },
    HuffmanEntry {
        bits: 15,
        code: 27903,
    },
    HuffmanEntry {
        bits: 15,
        code: 7423,
    },
    HuffmanEntry {
        bits: 15,
        code: 23807,
    },
    HuffmanEntry {
        bits: 15,
        code: 15615,
    },
    HuffmanEntry {
        bits: 15,
        code: 31999,
    },
    HuffmanEntry {
        bits: 15,
        code: 767,
    },
    HuffmanEntry {
        bits: 15,
        code: 17151,
    },
    HuffmanEntry {
        bits: 15,
        code: 8959,
    },
    HuffmanEntry {
        bits: 15,
        code: 25343,
    },
    HuffmanEntry {
        bits: 15,
        code: 4863,
    },
    HuffmanEntry {
        bits: 15,
        code: 21247,
    },
    HuffmanEntry {
        bits: 15,
        code: 13055,
    },
    HuffmanEntry {
        bits: 15,
        code: 29439,
    },
    HuffmanEntry {
        bits: 15,
        code: 2815,
    },
    HuffmanEntry {
        bits: 15,
        code: 19199,
    },
    HuffmanEntry {
        bits: 15,
        code: 11007,
    },
    HuffmanEntry {
        bits: 15,
        code: 27391,
    },
    HuffmanEntry {
        bits: 15,
        code: 6911,
    },
    HuffmanEntry {
        bits: 15,
        code: 23295,
    },
    HuffmanEntry {
        bits: 15,
        code: 15103,
    },
    HuffmanEntry {
        bits: 15,
        code: 31487,
    },
    HuffmanEntry {
        bits: 15,
        code: 1791,
    },
    HuffmanEntry {
        bits: 15,
        code: 18175,
    },
    HuffmanEntry {
        bits: 15,
        code: 9983,
    },
    HuffmanEntry {
        bits: 15,
        code: 26367,
    },
    HuffmanEntry {
        bits: 15,
        code: 5887,
    },
    HuffmanEntry {
        bits: 15,
        code: 22271,
    },
    HuffmanEntry {
        bits: 15,
        code: 14079,
    },
    HuffmanEntry {
        bits: 15,
        code: 30463,
    },
    HuffmanEntry {
        bits: 15,
        code: 3839,
    },
    HuffmanEntry {
        bits: 15,
        code: 20223,
    },
    HuffmanEntry {
        bits: 15,
        code: 12031,
    },
    HuffmanEntry {
        bits: 15,
        code: 28415,
    },
    HuffmanEntry {
        bits: 15,
        code: 7935,
    },
    HuffmanEntry {
        bits: 15,
        code: 24319,
    },
    HuffmanEntry {
        bits: 15,
        code: 16127,
    },
    HuffmanEntry {
        bits: 15,
        code: 32511,
    },
    HuffmanEntry {
        bits: 15,
        code: 511,
    },
    HuffmanEntry {
        bits: 15,
        code: 16895,
    },
    HuffmanEntry {
        bits: 15,
        code: 8703,
    },
    HuffmanEntry {
        bits: 15,
        code: 25087,
    },
    HuffmanEntry {
        bits: 15,
        code: 4607,
    },
    HuffmanEntry {
        bits: 15,
        code: 20991,
    },
    HuffmanEntry {
        bits: 15,
        code: 12799,
    },
    HuffmanEntry {
        bits: 15,
        code: 29183,
    },
    HuffmanEntry {
        bits: 15,
        code: 2559,
    },
    HuffmanEntry {
        bits: 15,
        code: 18943,
    },
    HuffmanEntry {
        bits: 15,
        code: 10751,
    },
    HuffmanEntry {
        bits: 15,
        code: 27135,
    },
    HuffmanEntry {
        bits: 15,
        code: 6655,
    },
    HuffmanEntry {
        bits: 15,
        code: 23039,
    },
    HuffmanEntry {
        bits: 15,
        code: 14847,
    },
    HuffmanEntry {
        bits: 15,
        code: 31231,
    },
    HuffmanEntry {
        bits: 15,
        code: 1535,
    },
    HuffmanEntry {
        bits: 15,
        code: 17919,
    },
    HuffmanEntry {
        bits: 15,
        code: 9727,
    },
    HuffmanEntry {
        bits: 15,
        code: 26111,
    },
    HuffmanEntry {
        bits: 15,
        code: 5631,
    },
    HuffmanEntry {
        bits: 15,
        code: 22015,
    },
    HuffmanEntry {
        bits: 15,
        code: 13823,
    },
    HuffmanEntry {
        bits: 15,
        code: 30207,
    },
    HuffmanEntry {
        bits: 15,
        code: 3583,
    },
    HuffmanEntry {
        bits: 15,
        code: 19967,
    },
    HuffmanEntry {
        bits: 15,
        code: 11775,
    },
    HuffmanEntry {
        bits: 15,
        code: 28159,
    },
    HuffmanEntry {
        bits: 15,
        code: 7679,
    },
    HuffmanEntry {
        bits: 15,
        code: 24063,
    },
    HuffmanEntry {
        bits: 15,
        code: 15871,
    },
    HuffmanEntry {
        bits: 15,
        code: 32255,
    },
    HuffmanEntry {
        bits: 15,
        code: 1023,
    },
    HuffmanEntry {
        bits: 15,
        code: 17407,
    },
    HuffmanEntry {
        bits: 15,
        code: 9215,
    },
    HuffmanEntry {
        bits: 15,
        code: 25599,
    },
    HuffmanEntry {
        bits: 15,
        code: 5119,
    },
    HuffmanEntry {
        bits: 15,
        code: 21503,
    },
    HuffmanEntry {
        bits: 15,
        code: 13311,
    },
    HuffmanEntry {
        bits: 15,
        code: 29695,
    },
    HuffmanEntry {
        bits: 15,
        code: 3071,
    },
    HuffmanEntry {
        bits: 15,
        code: 19455,
    },
    HuffmanEntry {
        bits: 15,
        code: 11263,
    },
    HuffmanEntry {
        bits: 15,
        code: 27647,
    },
    HuffmanEntry {
        bits: 15,
        code: 7167,
    },
    HuffmanEntry {
        bits: 15,
        code: 23551,
    },
    HuffmanEntry {
        bits: 15,
        code: 15359,
    },
    HuffmanEntry {
        bits: 15,
        code: 31743,
    },
    HuffmanEntry {
        bits: 15,
        code: 2047,
    },
    HuffmanEntry {
        bits: 15,
        code: 18431,
    },
    HuffmanEntry {
        bits: 15,
        code: 10239,
    },
    HuffmanEntry {
        bits: 15,
        code: 26623,
    },
    HuffmanEntry {
        bits: 15,
        code: 6143,
    },
    HuffmanEntry {
        bits: 15,
        code: 22527,
    },
    HuffmanEntry {
        bits: 15,
        code: 14335,
    },
    HuffmanEntry {
        bits: 15,
        code: 30719,
    },
    HuffmanEntry {
        bits: 15,
        code: 4095,
    },
    HuffmanEntry {
        bits: 15,
        code: 20479,
    },
    HuffmanEntry {
        bits: 15,
        code: 12287,
    },
    HuffmanEntry {
        bits: 15,
        code: 28671,
    },
    HuffmanEntry {
        bits: 15,
        code: 8191,
    },
    HuffmanEntry {
        bits: 15,
        code: 24575,
    },
    HuffmanEntry {
        bits: 15,
        code: 16383,
    },
    HuffmanEntry {
        bits: 14,
        code: 8831,
    },
    HuffmanEntry {
        bits: 12,
        code: 3199,
    },
    HuffmanEntry {
        bits: 11,
        code: 1471,
    },
    HuffmanEntry {
        bits: 11,
        code: 959,
    },
    HuffmanEntry {
        bits: 11,
        code: 1983,
    },
    HuffmanEntry {
        bits: 11,
        code: 127,
    },
    HuffmanEntry {
        bits: 10,
        code: 831,
    },
    HuffmanEntry {
        bits: 10,
        code: 191,
    },
    HuffmanEntry { bits: 9, code: 223 },
    HuffmanEntry { bits: 9, code: 479 },
    HuffmanEntry { bits: 8, code: 239 },
    HuffmanEntry { bits: 8, code: 31 },
    HuffmanEntry { bits: 8, code: 159 },
    HuffmanEntry { bits: 6, code: 55 },
    HuffmanEntry { bits: 7, code: 79 },
    HuffmanEntry { bits: 4, code: 3 },
    HuffmanEntry { bits: 4, code: 11 },
    HuffmanEntry { bits: 3, code: 1 },
    HuffmanEntry {
        bits: 15,
        code: 32767,
    },
];
