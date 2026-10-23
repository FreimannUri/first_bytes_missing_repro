//! Reproducer + config sweep: first bytes of each JPEG frame when capturing with
//! the async camera API and `EofMode::VsyncSignal` (ESP32-S3-EYE pinout, OV5640).
//!
//! Setup:
//!   * dependencies: same versions as your project (esp-hal with `unstable`,
//!     esp-rtos with embassy, embassy-executor, embassy-time, esp-println,
//!     esp-backtrace, esp-bootloader-esp-idf)
//!   * everything lives in this file: the OV5640 register setup is copied from
//!     the project's `ov5640.rs` (VGA JPEG path only, other resolutions removed)
//!   * no PSRAM, no flash tweaks: DMA buffers live in internal RAM
//!
//! For every variant in SWEEP the camera is reconfigured with `apply_config()`
//! and FRAMES frames are captured in a loop `receive() -> wait_for_dma_eof() ->
//! stop()`. Nothing is printed while capturing; results of a variant are printed
//! after its frames. The spare buffer is refilled with 0xA5 while the other
//! transfer is in flight, so the gap between `stop()` and the next `receive()`
//! contains only a few cheap reads of the finished buffer.
//!
//! How to read the output:
//!   * frame 0 of each variant is a warm-up: reception may start partway
//!     through a frame (see the docs of `wait_for_dma_eof`), ignore it
//!   * a JPEG stream starts with FF D8; if the sensor emits a standard JFIF
//!     header (FF D8 FF E0 00 10 'J' 'F' 'I' 'F'), "JFIF" must be at offset 6.
//!     If the first 4 bytes are missing it shows up at offset 2 and the
//!     buffer does not start with FF D8
//!   * `gap` is the time between `stop()` returning and the next `receive()`
//!     returning; `dt` is the time between consecutive EOFs (frame period)
//!   * a variant that produces no EOF is reported as FAILED and the sweep goes on
//!
//! The variant that matters for the upstream Issue is the first one (defaults).

#![no_std]
#![no_main]

use embassy_time::{Duration, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    delay::Delay,
    dma::DmaRxBuf,
    dma_rx_buffer,
    i2c::master::{Config as I2cConfig, Error as I2cError, I2c},
    lcd_cam::{
        LcdCam,
        cam::{Camera, Config as CamConfig, EofMode, VhdeMode, VsyncFilterThreshold},
    },
    time::{Instant, Rate},
    timer::timg::TimerGroup,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const FRAMES: usize = 7; // 1 warm-up + 6
const BUF_BYTES: usize = 64 * 1024; // must hold one VGA JPEG at QSCALE
const QSCALE: u8 = 0x0C;
const FILL: u8 = 0xA5;

const I2C_ADDR: u8 = 0x3C;

fn w8(i2c: &mut I2c<'_, Blocking>, reg: u16, v: u8) -> Result<(), I2cError> {
    i2c.write(I2C_ADDR, &[(reg >> 8) as u8, reg as u8, v])
}

fn w16(i2c: &mut I2c<'_, Blocking>, reg: u16, v: u16) -> Result<(), I2cError> {
    w8(i2c, reg, (v >> 8) as u8)?;
    w8(i2c, reg + 1, v as u8)
}

fn r8(i2c: &mut I2c<'_, Blocking>, reg: u16) -> Result<u8, I2cError> {
    let mut b = [0u8];
    i2c.write_read(I2C_ADDR, &[(reg >> 8) as u8, reg as u8], &mut b)?;
    Ok(b[0])
}

fn sensor_init(i2c: &mut I2c<'_, Blocking>) -> Result<(), I2cError> {
    let mut id_bytes = [0u8; 2];
    i2c.write_read(I2C_ADDR, &[0x30, 0x0A], &mut id_bytes)?;

    let read_id = ((id_bytes[0] as u16) << 8) | (id_bytes[1] as u16);
    if read_id != 0x5640 {
        panic!("unexpected sensor ID {:#06x} (expected 0x5640)", read_id);
    }
    println!("[sensor] OV5640 ID match: 0x{:04X}", read_id);

    const REG_DLY: u16 = 0xFFFF;

    const DEFAULT_REGS: &[[u16; 2]] = &[
        [0x3008, 0x82], // software reset
        [REG_DLY, 10],  // delay 10ms
        [0x3008, 0x42], // power down
        // enable pll
        [0x3103, 0x13],
        // io direction
        [0x3017, 0xFF],
        [0x3018, 0xFF],
        [0x3020, 0xC3], // DRIVE_CAPABILITY
        [0x3021, 0x21], // CLOCK_POL_CONTROL
        [0x4713, 0x02], // jpg mode select
        // [0x4713, 0x03],
        [0x5001, 0x83], // ISP_CONTROL_01: turn color matrix, awb and SDE
        // sys reset
        [0x3000, 0x20], // reset MCU
        [REG_DLY, 10],  // delay 10ms
        [0x3002, 0x1C],
        // clock enable
        [0x3004, 0xFF],
        [0x3006, 0xC3],
        // isp control
        [0x5000, 0xA7],
        [0x5001, 0xA3], // ISP_CONTROL_01 + scaling?
        [0x5003, 0x08], // special_effect
        // unknown
        [0x370C, 0x02], // !!IMPORTANT
        [0x3634, 0x40], // !!IMPORTANT
        // AEC/AGC
        [0x3A02, 0x03],
        [0x3A03, 0xD8],
        [0x3A08, 0x01],
        [0x3A09, 0x27],
        [0x3A0A, 0x00],
        [0x3A0B, 0xF6],
        [0x3A0D, 0x04],
        [0x3A0E, 0x03],
        [0x3A0F, 0x30], // ae_level
        [0x3A10, 0x28], // ae_level
        [0x3A11, 0x60], // ae_level
        [0x3A13, 0x43],
        [0x3A14, 0x03],
        [0x3A15, 0xD8],
        [0x3A18, 0x00], // gainceiling
        [0x3A19, 0xF8], // gainceiling
        [0x3A1B, 0x30], // ae_level
        [0x3A1E, 0x26], // ae_level
        [0x3A1F, 0x14], // ae_level
        // vcm debug
        [0x3600, 0x08],
        [0x3601, 0x33],
        // 50/60Hz
        [0x3C01, 0xA4],
        [0x3C04, 0x28],
        [0x3C05, 0x98],
        [0x3C06, 0x00],
        [0x3C07, 0x08],
        [0x3C08, 0x00],
        [0x3C09, 0x1C],
        [0x3C0A, 0x9C],
        [0x3C0B, 0x40],
        [0x460C, 0x22], // disable jpeg footer
        // BLC
        [0x4001, 0x02],
        [0x4004, 0x02],
        // AWB
        [0x5180, 0xFF],
        [0x5181, 0xF2],
        [0x5182, 0x00],
        [0x5183, 0x14],
        [0x5184, 0x25],
        [0x5185, 0x24],
        [0x5186, 0x09],
        [0x5187, 0x09],
        [0x5188, 0x09],
        [0x5189, 0x75],
        [0x518a, 0x54],
        [0x518b, 0xE0],
        [0x518c, 0xB2],
        [0x518d, 0x42],
        [0x518e, 0x3D],
        [0x518f, 0x56],
        [0x5190, 0x46],
        [0x5191, 0xF8],
        [0x5192, 0x04],
        [0x5193, 0x70],
        [0x5194, 0xF0],
        [0x5195, 0xF0],
        [0x5196, 0x03],
        [0x5197, 0x01],
        [0x5198, 0x04],
        [0x5199, 0x12],
        [0x519a, 0x04],
        [0x519b, 0x00],
        [0x519c, 0x06],
        [0x519d, 0x82],
        [0x519e, 0x38],
        // color matrix (Saturation)
        [0x5381, 0x1E],
        [0x5382, 0x5B],
        [0x5383, 0x08],
        [0x5384, 0x0A],
        [0x5385, 0x7E],
        [0x5386, 0x88],
        [0x5387, 0x7C],
        [0x5388, 0x6C],
        [0x5389, 0x10],
        [0x538a, 0x01],
        [0x538b, 0x98],
        // CIP control (Sharpness)
        [0x5300, 0x10], // sharpness
        [0x5301, 0x10], // sharpness
        [0x5302, 0x18], // sharpness
        [0x5303, 0x19], // sharpness
        [0x5304, 0x10],
        [0x5305, 0x10],
        [0x5306, 0x08], // denoise
        [0x5307, 0x16],
        [0x5308, 0x40],
        [0x5309, 0x10], // sharpness
        [0x530a, 0x10], // sharpness
        [0x530b, 0x04], // sharpness
        [0x530c, 0x06], // sharpness
        // GAMMA
        [0x5480, 0x01],
        [0x5481, 0x00],
        [0x5482, 0x1E],
        [0x5483, 0x3B],
        [0x5484, 0x58],
        [0x5485, 0x66],
        [0x5486, 0x71],
        [0x5487, 0x7D],
        [0x5488, 0x83],
        [0x5489, 0x8F],
        [0x548a, 0x98],
        [0x548b, 0xA6],
        [0x548c, 0xB8],
        [0x548d, 0xCA],
        [0x548e, 0xD7],
        [0x548f, 0xE3],
        [0x5490, 0x1D],
        // Special Digital Effects (SDE) (UV adjust)
        [0x5580, 0x06], // enable brightness and contrast
        [0x5583, 0x40], // special_effect
        [0x5584, 0x10], // special_effect
        [0x5586, 0x20], // contrast
        [0x5587, 0x00], // brightness
        [0x5588, 0x00], // brightness
        [0x5589, 0x10],
        [0x558a, 0x00],
        [0x558b, 0xF8],
        [0x501D, 0x40], // enable manual offset of contrast
        // power on
        [0x3008, 0x02],
        // 50Hz
        [0x3C00, 0x04],
        [REG_DLY, 300],
        // [REGLIST_TAIL, 0x00],
    ];

    let delay = Delay::new();

    for entry in DEFAULT_REGS {
        let reg = entry[0];
        let val = entry[1] as u8;

        if reg == REG_DLY {
            delay.delay_millis(entry[1] as u32);
        } else {
            let bytes = [(reg >> 8) as u8, reg as u8, val];
            i2c.write(I2C_ADDR, &bytes)?;
        }
    }

    Ok(())
}

fn sensor_set_vga_jpeg(i2c: &mut I2c<'_, Blocking>, qscale: u8) -> Result<(), I2cError> {
    // 1. JPEG output format
    for (r, v) in [
        (0x501Fu16, 0x00u8),
        (0x4300, 0x30),
        (0x3002, 0x00),
        (0x3006, 0xFF),
        (0x471C, 0x50),
    ] {
        w8(i2c, r, v)?;
    }

    // 2. window: 2560x1920 array, output 640x480 (binning)
    w16(i2c, 0x3800, 0)?; // X start
    w16(i2c, 0x3802, 0)?; // Y start
    w16(i2c, 0x3804, 2623)?; // X end
    w16(i2c, 0x3806, 1951)?; // Y end
    w16(i2c, 0x3808, 640)?; // output width
    w16(i2c, 0x380A, 480)?; // output height
    w16(i2c, 0x380C, 2060)?; // total X
    w16(i2c, 0x380E, 984)?; // total Y
    w16(i2c, 0x3810, 16)?; // offset X
    w16(i2c, 0x3812, 8)?; // offset Y
    let isp = r8(i2c, 0x5001)?;
    w8(i2c, 0x5001, isp | 0x20)?; // scaling enabled

    // 3. image options (no mirror/flip), JPEG bit 0x20 in 0x3821
    w8(i2c, 0x3820, 0x01)?;
    w8(i2c, 0x3821, 0x21)?;
    w8(i2c, 0x4514, 0xAA)?;
    w8(i2c, 0x4520, 0x0B)?;
    w8(i2c, 0x3814, 0x31)?;
    w8(i2c, 0x3815, 0x31)?;

    // 4. PLL for JPEG VGA
    w8(i2c, 0x3039, 0x00)?;
    w8(i2c, 0x3034, 0x1A)?;
    w8(i2c, 0x3035, 0x41)?;
    w8(i2c, 0x3036, 180)?; // multiplier
    w8(i2c, 0x3037, 0x02)?; // pre_div 2
    w8(i2c, 0x3108, 0x26)?;
    w8(i2c, 0x3824, 0x04)?; // pclk_div 4
    w8(i2c, 0x460C, 0x22)?;
    w8(i2c, 0x3103, 0x13)?;

    // 5. JPEG quality
    w8(i2c, 0x4407, qscale & 0x3F)?;

    for reg in [0x3002u16, 0x3006, 0x3821, 0x4713, 0x471C, 0x5001] {
        println!("[sensor] reg {:#06x} = {:#04x}", reg, r8(i2c, reg)?);
    }
    Ok(())
}

struct Variant {
    name: &'static str,
    filter: Option<VsyncFilterThreshold>,
    inv_vsync: bool,
    inv_de: bool,
}

const SWEEP: [Variant; 7] = [
    Variant {
        name: "defaults (De, no vsync filter)",
        filter: None,
        inv_vsync: false,
        inv_de: false,
    },
    Variant {
        name: "vsync filter One",
        filter: Some(VsyncFilterThreshold::One),
        inv_vsync: false,
        inv_de: false,
    },
    Variant {
        name: "vsync filter Four",
        filter: Some(VsyncFilterThreshold::Four),
        inv_vsync: false,
        inv_de: false,
    },
    Variant {
        name: "vsync filter Five",
        filter: Some(VsyncFilterThreshold::Five),
        inv_vsync: false,
        inv_de: false,
    },
    Variant {
        name: "invert_vsync",
        filter: None,
        inv_vsync: true,
        inv_de: false,
    },
    Variant {
        name: "invert_h_enable",
        filter: None,
        inv_vsync: false,
        inv_de: true,
    },
    Variant {
        name: "invert_vsync + invert_h_enable",
        filter: None,
        inv_vsync: true,
        inv_de: true,
    },
];

fn make_cfg(v: &Variant) -> CamConfig {
    let c = CamConfig::default()
        .with_frequency(Rate::from_mhz(20)) // XCLK
        .with_vh_de_mode(VhdeMode::De)
        .with_eof_mode(EofMode::VsyncSignal)
        .with_invert_vsync(v.inv_vsync)
        .with_invert_h_enable(v.inv_de);
    match v.filter {
        // NOTE: if the generated setter takes `Option<_>`, pass `Some(t)` here.
        Some(t) => c.with_vsync_filter_threshold(t),
        None => c,
    }
}

#[derive(Clone, Copy)]
struct Rec {
    rx: usize,
    head: [u8; 16],
    head_len: usize,
    jfif_at: Option<usize>,
    eoi_at: Option<usize>,
    gap_us: u64,
    dt_us: u64,
}

impl Rec {
    const EMPTY: Rec = Rec {
        rx: 0,
        head: [0; 16],
        head_len: 0,
        jfif_at: None,
        eoi_at: None,
        gap_us: 0,
        dt_us: 0,
    };
}

/// Cheap summary: reads only the head, the tail and the descriptor lengths.
fn summarize(buf: &DmaRxBuf) -> Rec {
    let all = buf.as_slice();
    let rx = buf.number_of_received_bytes().min(all.len());
    let d = &all[..rx];

    let mut r = Rec::EMPTY;
    r.rx = rx;
    r.head_len = d.len().min(16);
    r.head[..r.head_len].copy_from_slice(&d[..r.head_len]);
    r.jfif_at = d[..d.len().min(32)]
        .windows(4)
        .position(|w| w == &b"JFIF"[..]);
    let from = d.len().saturating_sub(64);
    r.eoi_at = d[from..]
        .windows(2)
        .rposition(|w| w == [0xFF, 0xD9])
        .map(|i| i + from);
    r
}

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut cur = dma_rx_buffer!(BUF_BYTES).unwrap();
    let mut other = dma_rx_buffer!(BUF_BYTES).unwrap();
    cur.as_mut_slice().fill(FILL);
    other.as_mut_slice().fill(FILL);

    // The camera peripheral generates XCLK, so it must exist before the
    // sensor is configured over I2C.
    let lcd_cam = LcdCam::new(p.LCD_CAM);
    let mut camera = Camera::new(lcd_cam.cam, p.DMA_CH0, make_cfg(&SWEEP[0]))
        .unwrap()
        .with_master_clock(p.GPIO15)
        .with_pixel_clock(p.GPIO13)
        .with_vsync(p.GPIO6)
        .with_h_enable(p.GPIO7) // HREF used as DE
        .with_data0(p.GPIO11)
        .with_data1(p.GPIO9)
        .with_data2(p.GPIO8)
        .with_data3(p.GPIO10)
        .with_data4(p.GPIO12)
        .with_data5(p.GPIO18)
        .with_data6(p.GPIO17)
        .with_data7(p.GPIO16)
        .into_async();

    let mut i2c = I2c::new(p.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(p.GPIO4)
        .with_scl(p.GPIO5);

    sensor_init(&mut i2c).unwrap();
    sensor_set_vga_jpeg(&mut i2c, QSCALE).unwrap();
    Delay::new().delay_millis(300);

    println!(
        "[repro] VGA JPEG q={:#04x}, XCLK 20 MHz, EofMode::VsyncSignal, async, {} frames/variant, buf {} bytes (internal RAM)",
        QSCALE, FRAMES, BUF_BYTES
    );

    let base = Instant::now();

    for v in SWEEP.iter() {
        camera.apply_config(&make_cfg(v)).unwrap();

        let mut recs = [Rec::EMPTY; FRAMES];
        let mut got = 0usize;
        let mut fail: Option<&'static str> = None;
        let mut t_stopped = 0u64;
        let mut t_eof_prev = 0u64;

        for i in 0..FRAMES {
            let mut transfer = camera
                .receive(cur)
                .map_err(|(e, _, _)| e)
                .expect("receive() failed");
            let t_armed = base.elapsed().as_micros();

            // Runs while the transfer is in flight, outside the inter-frame gap.
            other.as_mut_slice().fill(FILL);

            let eof = with_timeout(Duration::from_millis(1500), transfer.wait_for_dma_eof()).await;
            let t_eof = base.elapsed().as_micros();

            let (c, done) = transfer.stop();
            camera = c;
            let t_stopped_now = base.elapsed().as_micros();

            match eof {
                Ok(Ok(())) => {
                    let mut rec = summarize(&done);
                    if i > 0 {
                        rec.gap_us = t_armed.saturating_sub(t_stopped);
                        rec.dt_us = t_eof.saturating_sub(t_eof_prev);
                    }
                    recs[i] = rec;
                    got = i + 1;
                }
                Ok(Err(_)) => fail = Some("DMA error (frame larger than buffer?)"),
                Err(_) => fail = Some("no EOF within 1.5 s"),
            }
            t_stopped = t_stopped_now;
            t_eof_prev = t_eof;

            cur = other; // already refilled
            other = done; // refilled during the next iteration

            if fail.is_some() {
                break;
            }
        }

        println!("[repro] === {} ===", v.name);
        println!("[repro] idx    rx   gap_us    dt_us jfif@ eoi@  head[0..16]");
        for (i, r) in recs.iter().take(got).enumerate() {
            println!(
                "[repro] {:>3} {:>5} {:>8} {:>8} {:>5} {:>5?} {:02X?}{}",
                i,
                r.rx,
                r.gap_us,
                r.dt_us,
                match r.jfif_at {
                    Some(x) => x as i32,
                    None => -1,
                },
                r.eoi_at,
                &r.head[..r.head_len],
                if i == 0 { "  (warm-up)" } else { "" }
            );
        }
        if let Some(f) = fail {
            println!("[repro] FAILED at frame {}: {}", got, f);
        }

        let (mut n, mut soi, mut jfif6) = (0usize, 0usize, 0usize);
        for r in recs.iter().take(got).skip(1) {
            n += 1;
            if r.head_len >= 2 && r.head[0] == 0xFF && r.head[1] == 0xD8 {
                soi += 1;
            }
            if r.jfif_at == Some(6) {
                jfif6 += 1;
            }
        }
        println!(
            "[repro] SUMMARY [{}]: start with FF D8: {}/{}, JFIF at offset 6: {}/{}",
            v.name, soi, n, jfif6, n
        );
    }

    println!("[repro] SWEEP DONE");
    loop {
        Timer::after_secs(1).await;
    }
}
