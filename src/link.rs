use anyhow::Result;
use std::net::TcpStream;
use std::io::{Read, Write};
use std::time::Duration;
use serialport::SerialPort;

pub enum ReadState {
    Message(String),
    WouldBlock,
    Disconnected,
}

pub struct TcpLink {
    pub(crate) stream: TcpStream,
}

impl TcpLink {
    pub fn connect(addr: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect(addr)?;
        stream.set_nonblocking(true)?;
        Ok(TcpLink { stream })
    }

    pub fn send(&mut self, data: &str) -> std::io::Result<()> {
        self.stream.write_all(data.as_bytes())
    }

    pub fn try_read(&mut self) -> Result<ReadState, std::io::Error> {
        let mut buf = [0u8; 256];
        match self.stream.read(&mut buf) {
            Ok(0) => Ok(ReadState::Disconnected),
            Ok(n) => Ok(ReadState::Message(String::from_utf8_lossy(&buf[..n]).to_string())),
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(ReadState::WouldBlock),
            Err(e) => Err(e),
        }
    }
}

pub trait RobotLink {
    fn send(&mut self, msg: &str) -> Result<()>;
    fn recv(&mut self) -> Result<Option<String>>;
}

pub struct SerialLink {
    port: Box<dyn SerialPort>,
}

impl SerialLink {
    pub fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()?;
        Ok(Self { port })
    }
}

impl RobotLink for SerialLink {
    fn send(&mut self, msg: &str) -> Result<()> {
        use std::io::Write;
        self.port.write_all(msg.as_bytes())?;
        Ok(())
    }

    fn recv(&mut self) -> Result<Option<String>> {
        use std::io::Read;
        let mut buf = [0; 512];
        match self.port.read(&mut buf) {
            Ok(0) => Ok(None),
            Ok(n) => Ok(Some(String::from_utf8_lossy(&buf[..n]).into())),
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e.into()),
        }
    }
}