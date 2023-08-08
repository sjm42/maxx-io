// mypin.rs

#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq)]
pub enum MyPin {
    Quiz01 = 0x0000,
    Quiz02 = 0x0001,
    Quiz03 = 0x0002,
    Quiz04 = 0x0003,
    Quiz05 = 0x0004,
    Quiz06 = 0x0005,
    Quiz07 = 0x0006,
    Quiz08 = 0x0007,

    Quiz09 = 0x0008,
    Quiz10 = 0x0009,
    Quiz11 = 0x000a,
    Quiz12 = 0x000b,
    Quiz13 = 0x000c,
    Quiz14 = 0x000d,
    Quiz15 = 0x000e,
    Quiz16 = 0x000f,

    Quiz17 = 0x0100,
    Quiz18 = 0x0101,
    Quiz19 = 0x0102,
    Quiz20 = 0x0103,
    Quiz21 = 0x0104,
    Quiz22 = 0x0105,
    Quiz23 = 0x0106,
    Quiz24 = 0x0107,

    Map01_Tammisaari = 0x0108,
    Map02_Helsinki = 0x0109,
    Map03_Porvoo = 0x010a,
    Map04_Kotka = 0x010b,
    Map05_Turku = 0x010c,
    Map06_Lahti = 0x010d,
    Map07_Hämeenlinna = 0x010e,
    Map08_Lappeenranta = 0x010f,

    Map09_Rauma = 0x0200,
    Map10_Pori = 0x0201,
    Map11_Tampere = 0x0202,
    Map12_Mikkeli = 0x0203,
    Map13_Savonlinna = 0x0204,
    Map14_Varkaus = 0x0205,
    Map15_Jyväskylä = 0x0206,
    Map16_Vilppula = 0x0207,

    Map17_Isojoki = 0x0208,
    Map18_Joensuu = 0x0209,
    Map19_Ilomantsi = 0x020a,
    Map20_Kuopio = 0x020b,
    Map21_Viitasaari = 0x020c,
    Map22_Ähtäri = 0x020d,
    Map23_Seinäjoki = 0x020e,
    Map24_Vaasa = 0x020f,

    Map25_Kaustinen = 0x0300,
    Map26_Kokkola = 0x0301,
    Map27_Nivala = 0x0302,
    Map28_Iisalmi = 0x0303,
    Map29_Nurmes = 0x0304,
    Map30_Lieksa = 0x0305,
    Map31_Kuhmo = 0x0306,
    Map32_Kajaani = 0x0307,

    Map33_Raahe = 0x0308,
    Map34_Oulu = 0x0309,
    Map35_Suomussalmi = 0x030a,
    Map36_Pudasjärvi = 0x030b,
    Map37_Kemi = 0x030c,
    Map38_Aavasaksa = 0x030d,
    Map39_Kuusamo = 0x030e,
    Map40_Rovaniemi = 0x030f,

    Map41_Kemijärvi = 0x0508,
    Map42_Salla = 0x0509,
    Map43_Sodankylä = 0x050a,
    Map44_Muonio = 0x050b,
    Map45_Korvatunturi = 0x050c,
    Map46_Inari = 0x050d,
    Map47_Utsjoki = 0x050e,
    Map48_Kilpisjärvi = 0x050f,

    UnknownPin = 0xffff,
}

pub fn pin_ident(chip: u8, pin: u8) -> MyPin {
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
            _ => MyPin::UnknownPin,
        },

        0x05 => match pin {
            0x08 => MyPin::Map41_Kemijärvi,
            0x09 => MyPin::Map42_Salla,
            0x0a => MyPin::Map43_Sodankylä,
            0x0b => MyPin::Map44_Muonio,
            0x0c => MyPin::Map45_Korvatunturi,
            0x0d => MyPin::Map46_Inari,
            0x0e => MyPin::Map47_Utsjoki,
            0x0f => MyPin::Map48_Kilpisjärvi,

            _ => MyPin::UnknownPin,
        },

        _ => MyPin::UnknownPin,
    }
}

// EOF
