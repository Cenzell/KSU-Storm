use std::io::{Read, Write};
use std::net::TcpListener;
use std::thread;
use std::time::Duration;

struct JoystickData {
    lx: f32,
    ly: f32,
    rx: f32,
    ry: f32,
}

fn calculate_motor_speeds(data: &JoystickData) -> [f32; 4] {
    let mut motor1_speed = data.lx + data.ly + data.rx;
    let mut motor2_speed = -data.lx + data.ly - data.rx;
    let mut motor3_speed = -data.lx - data.ly + data.rx;
    let mut motor4_speed = data.lx - data.ly - data.rx;

    let max_speed = motor1_speed.abs()
        .max(motor2_speed.abs())
        .max(motor3_speed.abs())
        .max(motor4_speed.abs());

    if max_speed > 1.0 {
        motor1_speed /= max_speed;
        motor2_speed /= max_speed;
        motor3_speed /= max_speed;
        motor4_speed /= max_speed;
    }

    [motor1_speed, motor2_speed, motor3_speed, motor4_speed]
}

fn handle_command(cmd: &str) {
    let parts: Vec<&str> = cmd.trim().split_whitespace().collect();
    if let Some(&command_name) = parts.get(0) {
        match command_name {
            "BUTTON_PRESS" => {
                println!("Executing robot action!");
            }
            "PING" => {
                //println!("Heartbeat received!");
            }
            "JOYSTICKS" => {
                if let Some(values_str) = parts.get(1) {
                    let speeds: Vec<f32> = values_str
                        .split(',')
                        .filter_map(|s| s.parse().ok())
                        .collect();
                    
                    if speeds.len() == 4 {
                        let joystick_data = JoystickData {
                            lx: speeds[0],
                            ly: speeds[1],
                            rx: speeds[2],
                            ry: speeds[3],
                        };
                        let motor_speeds = calculate_motor_speeds(&joystick_data);
                        println!(
                            "Setting motor speeds: M1={}, M2={}, M3={}, M4={}",
                            motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]
                        );
                    } else {
                        eprintln!("Invalid number of joystick values: {}", values_str);
                    }
                }
            }
            _ => println!("Unknown command: {}", cmd),
        }
    }
}

// TCP Server
fn tcp_server() -> std::io::Result<()> {
    let listener = TcpListener::bind("0.0.0.0:5000")?;
    println!("Robot TCP listening on port 5000...");

    for stream in listener.incoming() {
        match stream {
            Ok(mut stream) => {
                println!("New client connected: {:?}", stream.peer_addr());
                thread::spawn(move || {
                    let mut buffer = Vec::new();
                    let mut temp_buf = [0; 512];
                    loop {
                        match stream.read(&mut temp_buf) {
                            Ok(0) => {
                                println!("Client disconnected.");
                                break;
                            }
                            Ok(n) => {
                                buffer.extend_from_slice(&temp_buf[..n]);
                                while let Some(pos) = buffer.iter().position(|&b| b == b'\n') {
                                    let msg = String::from_utf8_lossy(&buffer[..pos]).to_string();
                                    handle_command(&msg);
                                    if stream.write_all(b"ACK\n").is_err() {
                                        break; // Client disconnected while writing
                                    }
                                    buffer.drain(..=pos);
                                }
                            }
                            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                                // Preventt busy-waiting
                                thread::sleep(Duration::from_millis(10));
                            }
                            Err(e) => {
                                eprintln!("TCP Read Error: {}", e);
                                break;
                            }
                        }
                    }
                });
            }
            Err(e) => eprintln!("TCP listener error: {}", e),
        }
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    std::thread::spawn(|| {
        tcp_server().unwrap();
    });

    loop {
        std::thread::sleep(std::time::Duration::from_secs(3600));
    }
}