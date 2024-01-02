// mypin.rs

// We have 8*16 input pins and 8*16 output pins on two separate i2c buses.

// NOTE: we use hex encoding in enum values:
// 0x0008_050c means
// 00 = input chip address
// 08 = input pin#
// 05 = output chip address
// 0c = output pin#
//
// chip address range is 00..07 inclusive
// pin# range is 00..0f inclusive

#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum MyPin {
    Quiz01 = 0x0000_0400,
    Quiz02 = 0x0001_0401,
    Quiz03 = 0x0002_0402,
    Quiz04 = 0x0003_0403,
    Quiz05 = 0x0004_0404,
    Quiz06 = 0x0005_0405,
    Quiz07 = 0x0006_0406,
    Quiz08 = 0x0007_0407,

    Quiz09 = 0x0008_0408,
    Quiz10 = 0x0009_0409,
    Quiz11 = 0x000a_040a,
    Quiz12 = 0x000b_040b,
    Quiz13 = 0x000c_040c,
    Quiz14 = 0x000d_040d,
    Quiz15 = 0x000e_040e,
    Quiz16 = 0x000f_040f,

    Quiz17 = 0x0100_0500,
    Quiz18 = 0x0101_0501,
    Quiz19 = 0x0102_0502,
    Quiz20 = 0x0103_0503,
    Quiz21 = 0x0104_0504,
    Quiz22 = 0x0105_0505,
    Quiz23 = 0x0106_0506,
    Quiz24 = 0x0107_0507,

    Map01_Tammisaari = 0x0108_0000,
    Map02_Helsinki = 0x0109_0001,
    Map03_Porvoo = 0x010a_0002,
    Map04_Kotka = 0x010b_0003,
    Map05_Turku = 0x010c_0004,
    Map06_Lahti = 0x010d_0005,
    Map07_Hämeenlinna = 0x010e_0006,
    Map08_Lappeenranta = 0x010f_0007,

    Map09_Rauma = 0x0200_0008,
    Map10_Pori = 0x0201_0009,
    Map11_Tampere = 0x0202_000a,
    Map12_Mikkeli = 0x0203_000b,
    Map13_Savonlinna = 0x0204_000c,
    Map14_Varkaus = 0x0205_000d,
    Map15_Jyväskylä = 0x0206_000e,
    Map16_Vilppula = 0x0207_000f,

    Map17_Isojoki = 0x0208_0100,
    Map18_Joensuu = 0x0209_0101,
    Map19_Ilomantsi = 0x020a_0102,
    Map20_Kuopio = 0x020b_0103,
    Map21_Viitasaari = 0x020c_0104,
    Map22_Ähtäri = 0x020d_0105,
    Map23_Seinäjoki = 0x020e_0106,
    Map24_Vaasa = 0x020f_0107,

    Map25_Kaustinen = 0x0300_0108,
    Map26_Kokkola = 0x0301_0109,
    Map27_Nivala = 0x0302_010a,
    Map28_Iisalmi = 0x0303_010b,
    Map29_Nurmes = 0x0304_010c,
    Map30_Lieksa = 0x0305_010d,
    Map31_Kuhmo = 0x0306_010e,
    Map32_Kajaani = 0x0307_010f,

    Map33_Raahe = 0x0308_0200,
    Map34_Oulu = 0x0309_0201,
    Map35_Suomussalmi = 0x030a_0202,
    Map36_Pudasjärvi = 0x030b_0203,
    Map37_Kemi = 0x030c_0204,
    Map38_Aavasaksa = 0x030d_0205,
    Map39_Kuusamo = 0x030e_0206,
    Map40_Rovaniemi = 0x030f_0207,

    Map41_Kemijärvi = 0x0400_0208,
    Map42_Salla = 0x0401_0209,
    Map43_Sodankylä = 0x0402_020a,
    Map44_Muonio = 0x0403_020b,
    Map45_Korvatunturi = 0x0404_020c,
    Map46_Inari = 0x0405_020d,
    Map47_Utsjoki = 0x0406_020e,
    Map48_Kilpisjärvi = 0x0407_020f,

    Socket01 = 0x0408_0508,
    Socket02 = 0x0409_0509,
    Socket03 = 0x040a_050a,
    Socket04 = 0x040b_050b,
    Socket05 = 0x040c_050c,
    Socket06 = 0x040d_050d,
    Socket07 = 0x040e_050e,
    Socket08 = 0x040f_050f,

    Socket09 = 0x0500_0600,
    Socket10 = 0x0501_0601,
    Socket11 = 0x0502_0602,
    Socket12 = 0x0503_0603,
    Socket13 = 0x0504_0604,
    Socket14 = 0x0505_0605,
    Socket15 = 0x0506_0606,
    Socket16 = 0x0507_0607,

    Socket17 = 0x0508_0608,
    Socket18 = 0x0509_0609,
    Socket19 = 0x050a_060a,
    Socket20 = 0x050b_060b,
    Socket21 = 0x050c_060c,
    Socket22 = 0x050d_060d,
    Socket23 = 0x050e_060e,
    Socket24 = 0x050f_060f,

    Relay01 = 0xffff_0300, // output only
    Bell01 = 0xffff_0301,  // output only
    Mode01 = 0x070f_ffff,  // intput only

    UnknownPin = 0xffff_ffff,
}

pub const OUT_QUIZ: [MyPin; 24] = [
    MyPin::Quiz01,
    MyPin::Quiz02,
    MyPin::Quiz03,
    MyPin::Quiz04,
    MyPin::Quiz05,
    MyPin::Quiz06,
    MyPin::Quiz07,
    MyPin::Quiz08,
    MyPin::Quiz09,
    MyPin::Quiz10,
    MyPin::Quiz11,
    MyPin::Quiz12,
    MyPin::Quiz13,
    MyPin::Quiz14,
    MyPin::Quiz15,
    MyPin::Quiz16,
    MyPin::Quiz17,
    MyPin::Quiz18,
    MyPin::Quiz19,
    MyPin::Quiz20,
    MyPin::Quiz21,
    MyPin::Quiz22,
    MyPin::Quiz23,
    MyPin::Quiz24,
];

pub const OUT_MAP_S: [MyPin; 24] = [
    MyPin::Map01_Tammisaari,
    MyPin::Map02_Helsinki,
    MyPin::Map03_Porvoo,
    MyPin::Map04_Kotka,
    MyPin::Map05_Turku,
    MyPin::Map06_Lahti,
    MyPin::Map07_Hämeenlinna,
    MyPin::Map08_Lappeenranta,
    MyPin::Map09_Rauma,
    MyPin::Map10_Pori,
    MyPin::Map11_Tampere,
    MyPin::Map12_Mikkeli,
    MyPin::Map13_Savonlinna,
    MyPin::Map14_Varkaus,
    MyPin::Map15_Jyväskylä,
    MyPin::Map16_Vilppula,
    MyPin::Map17_Isojoki,
    MyPin::Map18_Joensuu,
    MyPin::Map19_Ilomantsi,
    MyPin::Map20_Kuopio,
    MyPin::Map21_Viitasaari,
    MyPin::Map22_Ähtäri,
    MyPin::Map23_Seinäjoki,
    MyPin::Map24_Vaasa,
];

pub const OUT_MAP_N: [MyPin; 24] = [
    MyPin::Map25_Kaustinen,
    MyPin::Map26_Kokkola,
    MyPin::Map27_Nivala,
    MyPin::Map28_Iisalmi,
    MyPin::Map29_Nurmes,
    MyPin::Map30_Lieksa,
    MyPin::Map31_Kuhmo,
    MyPin::Map32_Kajaani,
    MyPin::Map33_Raahe,
    MyPin::Map34_Oulu,
    MyPin::Map35_Suomussalmi,
    MyPin::Map36_Pudasjärvi,
    MyPin::Map37_Kemi,
    MyPin::Map38_Aavasaksa,
    MyPin::Map39_Kuusamo,
    MyPin::Map40_Rovaniemi,
    MyPin::Map41_Kemijärvi,
    MyPin::Map42_Salla,
    MyPin::Map43_Sodankylä,
    MyPin::Map44_Muonio,
    MyPin::Map45_Korvatunturi,
    MyPin::Map46_Inari,
    MyPin::Map47_Utsjoki,
    MyPin::Map48_Kilpisjärvi,
];

pub const OUT_SOCKET: [MyPin; 24] = [
    MyPin::Socket01,
    MyPin::Socket02,
    MyPin::Socket03,
    MyPin::Socket04,
    MyPin::Socket05,
    MyPin::Socket06,
    MyPin::Socket07,
    MyPin::Socket08,
    MyPin::Socket09,
    MyPin::Socket10,
    MyPin::Socket11,
    MyPin::Socket12,
    MyPin::Socket13,
    MyPin::Socket14,
    MyPin::Socket15,
    MyPin::Socket16,
    MyPin::Socket17,
    MyPin::Socket18,
    MyPin::Socket19,
    MyPin::Socket20,
    MyPin::Socket21,
    MyPin::Socket22,
    MyPin::Socket23,
    MyPin::Socket24,
];

pub const ANSWERS_MAP_A: [Option<MyPin>; 24] = [
    Some(MyPin::Map22_Ähtäri),
    Some(MyPin::Map42_Salla),
    Some(MyPin::Map13_Savonlinna),
    Some(MyPin::Map26_Kokkola),
    Some(MyPin::Map46_Inari),
    Some(MyPin::Map17_Isojoki),
    Some(MyPin::Map38_Aavasaksa),
    Some(MyPin::Map03_Porvoo),
    Some(MyPin::Map35_Suomussalmi),
    Some(MyPin::Map27_Nivala),
    Some(MyPin::Map39_Kuusamo),
    Some(MyPin::Map37_Kemi),
    Some(MyPin::Map40_Rovaniemi),
    Some(MyPin::Map47_Utsjoki),
    Some(MyPin::Map33_Raahe),
    Some(MyPin::Map14_Varkaus),
    Some(MyPin::Map06_Lahti),
    Some(MyPin::Map43_Sodankylä),
    Some(MyPin::Map15_Jyväskylä),
    Some(MyPin::Map05_Turku),
    Some(MyPin::Map11_Tampere),
    Some(MyPin::Map34_Oulu),
    Some(MyPin::Map29_Nurmes),
    Some(MyPin::Map18_Joensuu),
];

pub const ANSWERS_MAP_B: [Option<MyPin>; 24] = [
    Some(MyPin::Map23_Seinäjoki),
    Some(MyPin::Map41_Kemijärvi),
    Some(MyPin::Map12_Mikkeli),
    Some(MyPin::Map19_Ilomantsi),
    Some(MyPin::Map07_Hämeenlinna),
    Some(MyPin::Map45_Korvatunturi),
    Some(MyPin::Map24_Vaasa),
    Some(MyPin::Map09_Rauma),
    Some(MyPin::Map25_Kaustinen),
    Some(MyPin::Map32_Kajaani),
    Some(MyPin::Map08_Lappeenranta),
    Some(MyPin::Map02_Helsinki),
    Some(MyPin::Map20_Kuopio),
    Some(MyPin::Map48_Kilpisjärvi),
    Some(MyPin::Map31_Kuhmo),
    Some(MyPin::Map28_Iisalmi),
    Some(MyPin::Map04_Kotka),
    Some(MyPin::Map16_Vilppula),
    Some(MyPin::Map44_Muonio),
    Some(MyPin::Map30_Lieksa),
    Some(MyPin::Map21_Viitasaari),
    Some(MyPin::Map36_Pudasjärvi),
    Some(MyPin::Map10_Pori),
    Some(MyPin::Map01_Tammisaari),
];

pub const ANSWERS_QUIZ: [Option<MyPin>; 24] = [
    Some(MyPin::Quiz14), // * Neuvostoliitto
    Some(MyPin::Quiz11), // * Japani
    Some(MyPin::Quiz08), // * Turkki
    Some(MyPin::Quiz09), // * USA
    Some(MyPin::Quiz19), // * Suomi
    Some(MyPin::Quiz22), // * Ranska
    Some(MyPin::Quiz01), // * Belgia
    Some(MyPin::Quiz24), // * Sveitsi
    Some(MyPin::Quiz15), // * DDR
    Some(MyPin::Quiz03), // * Länsi-Saksa
    Some(MyPin::Quiz20), // * Ruåtsi
    Some(MyPin::Quiz17), // * Islanti
    Some(MyPin::Quiz07), // * Englanti
    Some(MyPin::Quiz18), // * Norja
    Some(MyPin::Quiz13), // * Tsekkoslovakia
    Some(MyPin::Quiz02), // * Uganda
    Some(MyPin::Quiz23), // * Kreikka
    Some(MyPin::Quiz21), // * Kiina
    Some(MyPin::Quiz06), // * Romania
    Some(MyPin::Quiz10), // * Kanada
    Some(MyPin::Quiz12), // * Intia
    Some(MyPin::Quiz04), // * Unkari
    Some(MyPin::Quiz16), // * Tanska
    Some(MyPin::Quiz05), // * Espanja
];

pub fn socket_index(pin: MyPin) -> Option<usize> {
    match pin {
        MyPin::Socket01 => Some(0),
        MyPin::Socket02 => Some(1),
        MyPin::Socket03 => Some(2),
        MyPin::Socket04 => Some(3),
        MyPin::Socket05 => Some(4),
        MyPin::Socket06 => Some(5),
        MyPin::Socket07 => Some(6),
        MyPin::Socket08 => Some(7),

        MyPin::Socket09 => Some(8),
        MyPin::Socket10 => Some(9),
        MyPin::Socket11 => Some(10),
        MyPin::Socket12 => Some(11),
        MyPin::Socket13 => Some(12),
        MyPin::Socket14 => Some(13),
        MyPin::Socket15 => Some(14),
        MyPin::Socket16 => Some(15),

        MyPin::Socket17 => Some(16),
        MyPin::Socket18 => Some(17),
        MyPin::Socket19 => Some(18),
        MyPin::Socket20 => Some(19),
        MyPin::Socket21 => Some(20),
        MyPin::Socket22 => Some(21),
        MyPin::Socket23 => Some(22),
        MyPin::Socket24 => Some(23),
        _ => None,
    }
}

pub fn pin_input_ident(chip: u8, pin: u8) -> MyPin {
    match chip {
        0x00 => match pin {
            0x00 => MyPin::Quiz01,
            0x01 => MyPin::Quiz02,
            0x02 => MyPin::Quiz03,
            0x03 => MyPin::Quiz04,
            0x04 => MyPin::Quiz05,
            0x05 => MyPin::Quiz06,
            0x06 => MyPin::Quiz07,
            0x07 => MyPin::Quiz08,

            0x08 => MyPin::Quiz09,
            0x09 => MyPin::Quiz10,
            0x0a => MyPin::Quiz11,
            0x0b => MyPin::Quiz12,
            0x0c => MyPin::Quiz13,
            0x0d => MyPin::Quiz14,
            0x0e => MyPin::Quiz15,
            0x0f => MyPin::Quiz16,

            _ => MyPin::UnknownPin,
        },
        0x01 => match pin {
            0x00 => MyPin::Quiz17,
            0x01 => MyPin::Quiz18,
            0x02 => MyPin::Quiz19,
            0x03 => MyPin::Quiz20,
            0x04 => MyPin::Quiz21,
            0x05 => MyPin::Quiz22,
            0x06 => MyPin::Quiz23,
            0x07 => MyPin::Quiz24,

            0x08 => MyPin::Map01_Tammisaari,
            0x09 => MyPin::Map02_Helsinki,
            0x0a => MyPin::Map03_Porvoo,
            0x0b => MyPin::Map04_Kotka,
            0x0c => MyPin::Map05_Turku,
            0x0d => MyPin::Map06_Lahti,
            0x0e => MyPin::Map07_Hämeenlinna,
            0x0f => MyPin::Map08_Lappeenranta,

            _ => MyPin::UnknownPin,
        },
        0x02 => match pin {
            0x00 => MyPin::Map09_Rauma,
            0x01 => MyPin::Map10_Pori,
            0x02 => MyPin::Map11_Tampere,
            0x03 => MyPin::Map12_Mikkeli,
            0x04 => MyPin::Map13_Savonlinna,
            0x05 => MyPin::Map14_Varkaus,
            0x06 => MyPin::Map15_Jyväskylä,
            0x07 => MyPin::Map16_Vilppula,

            0x08 => MyPin::Map17_Isojoki,
            0x09 => MyPin::Map18_Joensuu,
            0x0a => MyPin::Map19_Ilomantsi,
            0x0b => MyPin::Map20_Kuopio,
            0x0c => MyPin::Map21_Viitasaari,
            0x0d => MyPin::Map22_Ähtäri,
            0x0e => MyPin::Map23_Seinäjoki,
            0x0f => MyPin::Map24_Vaasa,

            _ => MyPin::UnknownPin,
        },
        0x03 => match pin {
            0x00 => MyPin::Map25_Kaustinen,
            0x01 => MyPin::Map26_Kokkola,
            0x02 => MyPin::Map27_Nivala,
            0x03 => MyPin::Map28_Iisalmi,
            0x04 => MyPin::Map29_Nurmes,
            0x05 => MyPin::Map30_Lieksa,
            0x06 => MyPin::Map31_Kuhmo,
            0x07 => MyPin::Map32_Kajaani,

            0x08 => MyPin::Map33_Raahe,
            0x09 => MyPin::Map34_Oulu,
            0x0a => MyPin::Map35_Suomussalmi,
            0x0b => MyPin::Map36_Pudasjärvi,
            0x0c => MyPin::Map37_Kemi,
            0x0d => MyPin::Map38_Aavasaksa,
            0x0e => MyPin::Map39_Kuusamo,
            0x0f => MyPin::Map40_Rovaniemi,

            _ => MyPin::UnknownPin,
        },

        0x04 => match pin {
            0x00 => MyPin::Map41_Kemijärvi,
            0x01 => MyPin::Map42_Salla,
            0x02 => MyPin::Map43_Sodankylä,
            0x03 => MyPin::Map44_Muonio,
            0x04 => MyPin::Map45_Korvatunturi,
            0x05 => MyPin::Map46_Inari,
            0x06 => MyPin::Map47_Utsjoki,
            0x07 => MyPin::Map48_Kilpisjärvi,

            0x08 => MyPin::Socket01,
            0x09 => MyPin::Socket02,
            0x0a => MyPin::Socket03,
            0x0b => MyPin::Socket04,
            0x0c => MyPin::Socket05,
            0x0d => MyPin::Socket06,
            0x0e => MyPin::Socket07,
            0x0f => MyPin::Socket08,

            _ => MyPin::UnknownPin,
        },

        0x05 => match pin {
            0x00 => MyPin::Socket09,
            0x01 => MyPin::Socket10,
            0x02 => MyPin::Socket11,
            0x03 => MyPin::Socket12,
            0x04 => MyPin::Socket13,
            0x05 => MyPin::Socket14,
            0x06 => MyPin::Socket15,
            0x07 => MyPin::Socket16,

            0x08 => MyPin::Socket17,
            0x09 => MyPin::Socket18,
            0x0a => MyPin::Socket19,
            0x0b => MyPin::Socket20,
            0x0c => MyPin::Socket21,
            0x0d => MyPin::Socket22,
            0x0e => MyPin::Socket23,
            0x0f => MyPin::Socket24,

            _ => MyPin::UnknownPin,
        },

        0x07 => match pin {
            0x0f => MyPin::Mode01,
            _ => MyPin::UnknownPin,
        },

        _ => MyPin::UnknownPin,
    }
}

// EOF
