#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{
    error::Error,
    sync::{Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use gilrs::{Axis, EventType, Gilrs};
use slint::{Timer, TimerMode};

mod link;
use link::{ReadState, TcpLink};

slint::include_modules!();

struct JoystickData {
    lx: f32,
    ly: f32,
    rx: f32,
    ry: f32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut gilrs = Gilrs::new()?;

    let ui = AppWindow::new()?;
    let ui_weak = ui.as_weak();

    // Shared TCP link
    let link: Arc<Mutex<Option<TcpLink>>> = Arc::new(Mutex::new(None));

    let addresses = vec![
        "127.0.0.1:5000".to_string(),
        "10.42.0.85:5000".to_string(), // Direct ethernet (Shared Network Linux) ip
    ];

    // Connection monitor & auto-reconnect
    let link_clone_conn = link.clone();
    let ui_clone_conn = ui_weak.clone();
    let addresses_clone = addresses.clone();
    thread::spawn(move || {
        let mut last_state = false;
        let mut current_address_index = 0;
        loop {
            let mut guard = link_clone_conn.lock().unwrap();

            let connected = if let Some(ref mut link) = *guard {
                match link.try_read() {
                    Ok(ReadState::Message(_)) | Ok(ReadState::WouldBlock) => true,
                    Ok(ReadState::Disconnected) | Err(_) => {
                        *guard = None;
                        false
                    }
                }
            } else {
                let addr = &addresses_clone[current_address_index];
                println!("Trying to connect to: {}", addr);
                match TcpLink::connect(addr) {
                    Ok(new_link) => {
                        *guard = Some(new_link);
                        true
                    }
                    Err(_) => {
                        current_address_index = (current_address_index + 1) % addresses_clone.len();
                        false
                    }
                }
            };

            if connected != last_state {
                let ui_clone_for_invoke = ui_clone_conn.clone();
                slint::invoke_from_event_loop(move || {
                    if let Some(ui) = ui_clone_for_invoke.upgrade() {
                        ui.set_connected(connected);
                    }
                }).ok();
                last_state = connected;
            }

            drop(guard);
            thread::sleep(Duration::from_secs(2));
        }
    });

    // Telemetry receive thread
    let link_for_rx = link.clone();
    let ui_for_rx = ui_weak.clone();
    thread::spawn(move || {
        loop {
            if let Ok(mut guard) = link_for_rx.lock() {
                if let Some(link) = guard.as_mut() {
                    if let Ok(ReadState::Message(msg)) = link.try_read() {
                        let ui_for_rx_clone = ui_for_rx.clone();
                        slint::invoke_from_event_loop(move || {
                            if let Some(ui) = ui_for_rx_clone.upgrade() {
                                ui.set_telemetry(msg.into());
                            }
                        }).ok();
                    }
                }
            }
            thread::sleep(Duration::from_millis(50));
        }
    });

    // Gamepad polling timer
    let link_for_timer = link.clone();
    let ui_timer_clone = ui_weak.clone();
    let mut last_ping_time = Instant::now();
    let ping_interval = Duration::from_secs(1);
    let timer = Timer::default();
    timer.start(
        TimerMode::Repeated,
        Duration::from_millis(16),
        move || {
            if last_ping_time.elapsed() >= ping_interval {
                send_to_robot(&link_for_timer, &ui_timer_clone, "PING\n".to_string());
                last_ping_time = Instant::now();
            }

            let mut joystick_values = JoystickData {
                lx: 0.0,
                ly: 0.0,
                rx: 0.0,
                ry: 0.0,
            };
            
            let mut event_occurred = false;

            while let Some(ev) = gilrs.next_event() {
                if let Some(app) = ui_timer_clone.upgrade() {
                    event_occurred = true;

                    match ev.event {
                        EventType::AxisChanged(Axis::LeftStickX, v, _) => {
                            app.set_lx(v);
                            joystick_values.lx = v;
                        }
                        EventType::AxisChanged(Axis::LeftStickY, v, _) => {
                            app.set_ly(-v);
                            joystick_values.ly = -v;
                        }
                        EventType::AxisChanged(Axis::RightStickX, v, _) => {
                            app.set_rx(v);
                            joystick_values.rx = v;
                        }
                        EventType::AxisChanged(Axis::RightStickY, v, _) => {
                            app.set_ry(-v);
                            joystick_values.ry = -v;
                        }
                        EventType::ButtonPressed(btn, _) => {
                            set_button(&app, btn, true);
                            send_to_robot(&link_for_timer, &ui_timer_clone, format!("BTN {:?} DOWN\n", btn));
                        }
                        EventType::ButtonReleased(btn, _) => {
                            set_button(&app, btn, false);
                            send_to_robot(&link_for_timer, &ui_timer_clone, format!("BTN {:?} UP\n", btn));
                        }
                        _ => {}
                    }
                }
            }
            
            if event_occurred {
                //let motor_speeds = calculate_motor_speeds(&joystick_values);
                send_to_robot(
                    &link_for_timer,
                    &ui_timer_clone,
                    format!(
                        "JOYSTICKS {},{},{},{}\n",
                        joystick_values.lx,
                        joystick_values.ly,
                        joystick_values.rx,
                        joystick_values.ry
                    ),
                );
            }
        },
    );

    ui.run()?;
    Ok(())
}

fn send_to_robot(
    link_arc: &Arc<Mutex<Option<TcpLink>>>,
    ui_weak: &slint::Weak<AppWindow>,
    message: String,
) {
    if let Ok(mut guard) = link_arc.lock() {
        if let Some(link) = guard.as_mut() {
            if link.send(&message).is_err() {
                *guard = None;
                let ui_weak = ui_weak.clone();
                slint::invoke_from_event_loop(move || {
                    if let Some(ui) = ui_weak.upgrade() {
                        ui.set_connected(false);
                    }
                }).ok();
            }
        }
    }
}

fn set_button(app: &AppWindow, btn: gilrs::Button, pressed: bool) {
    match btn {
        gilrs::Button::South => app.set_btn_cross(pressed),
        gilrs::Button::East => app.set_btn_circle(pressed),
        gilrs::Button::West => app.set_btn_square(pressed),
        gilrs::Button::North => app.set_btn_triangle(pressed),
        gilrs::Button::LeftTrigger => app.set_btn_l1(pressed),
        gilrs::Button::RightTrigger => app.set_btn_r1(pressed),
        _ => {}
    }
}