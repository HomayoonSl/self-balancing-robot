use embedded_hal::spi::SpiDevice;
use embedded_hal::delay::DelayNs;
use embedded_sdmmc::{TimeSource, Timestamp, VolumeManager, Mode, SdCard, VolumeIdx};

pub struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 54,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct SdLogger<DEV, DELAY, const N: usize> {
    spi_dev: DEV,
    delay: DELAY,
    _marker: core::marker::PhantomData<[(); N]>,
}

impl<DEV, DELAY, const N: usize> SdLogger<DEV, DELAY, N>
where
    DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    pub fn new(spi_dev: DEV, delay: DELAY) -> Self {
        Self {
            spi_dev,
            delay,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn dump_blackbox(
        &mut self,
        bb: &crate::utils::BlackBox<N>,
        dt_ms: u32,
    ) -> Result<(), &'static str> {
        let sdcard = SdCard::new(&mut self.spi_dev, &mut self.delay);
        let volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);

        let volume = volume_mgr
            .open_volume(VolumeIdx(0))
            .map_err(|_| "VolumeInitFailed")?;
            
        let root_dir = volume
            .open_root_dir()
            .map_err(|_| "OpenRootDirFailed")?;

        let file = root_dir
            .open_file_in_dir("log.csv", Mode::ReadWriteCreateOrTruncate)
            .map_err(|_| "FileOpenFailed")?;

        let header = b"row,pitch_cdeg,rate_ddps,wheel_crad,wheel_rate_crad_s,v_mv\r\n";
        file.write(header).map_err(|_| "WriteFailed")?;

        let start = (bb.head + N - bb.filled) % N;
        for i in 0..bb.filled {
            let r = bb.buf[(start + i) % N];
            let mut buf = [0u8; 64];
            let len = format_row(i, r, dt_ms, &mut buf);
            file.write(&buf[..len]).map_err(|_| "WriteFailed")?;
        }

        file.close().map_err(|_| "CloseFailed")?;
        Ok(())
    }
}

fn format_row(i: usize, r: crate::utils::blackbox::Rec, _dt_ms: u32, buf: &mut [u8]) -> usize {
    use core::fmt::Write;
    struct BufWriter<'a> {
        buf: &'a mut [u8],
        pos: usize,
    }
    impl<'a> Write for BufWriter<'a> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            if self.pos + bytes.len() > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
            self.pos += bytes.len();
            Ok(())
        }
    }

    let mut writer = BufWriter { buf, pos: 0 };
    let _ = write!(
        writer,
        "{},{},{},{},{},{}\r\n",
        i, r.pitch_cd, r.rate_dd, r.wheel_cr, r.wheel_rate_cr, r.v_mv
    );
    writer.pos
}
