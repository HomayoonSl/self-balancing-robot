use embedded_hal::spi::SpiDevice;
use embedded_hal::delay::DelayNs;
use embedded_sdmmc::{TimeSource, Timestamp, VolumeManager, Mode, SdCard, VolumeIdx, RawFile, RawDirectory, RawVolume};
use crate::utils::blackbox::Rec;

pub struct DummyTimeSource;

impl TimeSource for DummyTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 9,
            zero_indexed_day: 23,
            hours: 12,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub struct StreamLogger<DEV, DELAY>
where
    DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    volume_mgr: VolumeManager<SdCard<DEV, DELAY>, DummyTimeSource>,
    volume: Option<RawVolume>,
    root_dir: Option<RawDirectory>,
    file: Option<RawFile>,
}

impl<DEV, DELAY> StreamLogger<DEV, DELAY>
where
    DEV: SpiDevice<u8>,
    DELAY: DelayNs,
{
    pub fn new(spi_dev: DEV, delay: DELAY) -> Self {
        let sdcard = SdCard::new(spi_dev, delay);
        let volume_mgr = VolumeManager::new(sdcard, DummyTimeSource);
        Self {
            volume_mgr,
            volume: None,
            root_dir: None,
            file: None,
        }
    }

    pub fn open_session(&mut self) -> Result<(), &'static str> {
        let volume = self.volume_mgr.open_raw_volume(VolumeIdx(0)).map_err(|_| "VolOpen")?;
        let root_dir = self.volume_mgr.open_root_dir(volume).map_err(|_| "DirOpen")?;
        let file = self.volume_mgr.open_file_in_dir(
            root_dir,
            "log.csv",
            Mode::ReadWriteCreateOrTruncate,
        ).map_err(|_| "FileCreate")?;

        let header = b"row,pitch_cdeg,rate_ddps,wheel_crad,wheel_rate_crad_s,v_mv\r\n";
        self.volume_mgr.write(file, header).map_err(|_| "HeaderWrite")?;

        self.volume = Some(volume);
        self.root_dir = Some(root_dir);
        self.file = Some(file);
        Ok(())
    }

    pub fn log_record(&mut self, row_idx: u32, r: &Rec) -> Result<(), &'static str> {
        if let Some(file) = self.file {
            let mut buf = [0u8; 64];
            let len = format_row(row_idx, *r, &mut buf);
            self.volume_mgr.write(file, &buf[..len]).map_err(|_| "RowWrite")?;
            Ok(())
        } else {
            Err("FileNotOpen")
        }
    }

    pub fn close_session(&mut self) {
        if let Some(file) = self.file.take() {
            let _ = self.volume_mgr.close_file(file);
        }
        if let Some(dir) = self.root_dir.take() {
            let _ = self.volume_mgr.close_dir(dir);
        }
        if let Some(vol) = self.volume.take() {
            let _ = self.volume_mgr.close_volume(vol);
        }
    }
}

fn format_row(i: u32, r: Rec, buf: &mut [u8]) -> usize {
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
