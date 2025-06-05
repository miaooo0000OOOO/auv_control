use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

use serde::{Deserialize, Serialize};
use tauri::Manager;

#[derive(Serialize, Deserialize)]
struct AuvConfig {
    #[serde(rename = "auvHost")]
    auv_host: String,
    #[serde(rename = "videoStreamPort")]
    video_stream_port: u16,
    #[serde(rename = "videoStreamPath")]
    video_stream_path: String,
    #[serde(rename = "telemetryPort")]
    telemetry_port: u16,
    #[serde(rename = "telemetryPath")]
    telemetry_path: String,
    #[serde(rename = "controlMessagesPort")]
    control_messages_port: u16,
    #[serde(rename = "controlMessagesPath")]
    control_messages_path: String,
}

#[tauri::command]
async fn load_auv_config(app: tauri::AppHandle) -> Result<AuvConfig, String> {
    let config_path = app
        .path()
        .config_dir()
        .map_err(|e| format!("Failed to find config directory: {}", e))?
        .join("auv-gui")
        .join("auv_net_config.json");

    let config_data = tokio::fs::read_to_string(&config_path)
        .await
        .map_err(|e| format!("Failed to read config file: {}", e))?;

    let config: AuvConfig = serde_json::from_str(&config_data)
        .map_err(|e| format!("Failed to parse config file: {}", e))?;

    Ok(config)
}

#[tauri::command]
async fn write_auv_config(app: tauri::AppHandle, config: AuvConfig) -> Result<(), String> {
    let config_path = app
        .path()
        .config_dir()
        .map_err(|e| e.to_string())?
        .join("auv-gui")
        .join("auv_config.json");

    let config_dir = config_path.parent().ok_or("Invalid config path")?;
    tokio::fs::create_dir_all(config_dir)
        .await
        .map_err(|e| format!("Failed to create config directory: {}", e))?;

    let config_data = serde_json::to_string_pretty(&config)
        .map_err(|e| format!("Failed to serialize config: {}", e))?;

    tokio::fs::write(&config_path, config_data)
        .await
        .map_err(|e| format!("Failed to write config file: {}", e))?;

    Ok(())
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .setup(|app| {
            let app_handle = app.handle().clone();
            tauri::async_runtime::spawn(async move {
                // Load config using the app handle
                let auv_config = load_auv_config(app_handle)
                    .await
                    .expect("Failed to load AUV config");
                let port = auv_config.control_messages_port;
                let path = auv_config.control_messages_path.clone();
                start_ws_server(port, path).await;
            });
            Ok(())
        })
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![
            send_joystick,
            load_auv_config,
            write_auv_config
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}

use once_cell::sync::Lazy;
use std::sync::Mutex;

// 用于广播摇杆数据
static SENDER: Lazy<Mutex<Option<broadcast::Sender<String>>>> = Lazy::new(|| Mutex::new(None));

#[tauri::command]
fn send_joystick(data: String) {
    if let Some(sender) = SENDER.lock().unwrap().as_ref() {
        let _ = sender.send(data).unwrap_or_else(|e| {
            eprintln!("Failed to send joystick data: {}", e);
            0
        });
    }
}

async fn start_ws_server(port: u16, path: String) {
    use futures_util::sink::SinkExt;
    use futures_util::StreamExt;
    use tokio::net::TcpListener;
    use tokio_tungstenite::accept_hdr_async;
    use tokio_tungstenite::tungstenite::handshake::server::{Request, Response};

    let addr = format!("0.0.0.0:{}", port);
    let listener = TcpListener::bind(&addr).await.expect("Can't bind");
    println!("WebSocket server listening on {}{}", addr, path);

    let (tx, _) = broadcast::channel::<String>(16);
    *SENDER.lock().unwrap() = Some(tx.clone());

    loop {
        let (stream, _) = listener.accept().await.expect("Failed to accept");
        let tx = tx.clone();
        let path_clone = path.clone();

        tokio::spawn(async move {
            let callback = |req: &Request, resp: Response| {
                // 只允许指定路径
                let uri = req.uri().path();
                if uri == path_clone {
                    Ok(resp)
                } else {
                    Err(
                        tokio_tungstenite::tungstenite::handshake::server::ErrorResponse::new(
                            Some("Invalid path".into()),
                        ),
                    )
                }
            };

            let ws_stream = accept_hdr_async(stream, callback).await;
            if let Ok(ws_stream) = ws_stream {
                let (ws_sender, ws_receiver) = ws_stream.split();
                let mut rx = tx.subscribe();
                use std::sync::Arc;
                use tokio::sync::Mutex as TokioMutex;
                
                let ws_sender = Arc::new(TokioMutex::new(ws_sender));
                let ws_sender_clone = ws_sender.clone();
                
                tokio::spawn(async move {
                    let mut ws_receiver = ws_receiver;
                    let ws_sender = ws_sender_clone;
                    // 处理客户端发来的ping/pong/close等
                    while let Some(Ok(msg)) = ws_receiver.next().await {
                        match msg {
                            Message::Ping(payload) => {
                                let _ = ws_sender.lock().await.send(Message::Pong(payload)).await;
                            }
                            Message::Close(_) => {
                                break;
                            }
                            _ => {}
                        }
                    }
                });
                
                // 发送广播消息
                while let Ok(msg) = rx.recv().await {
                    let _ = ws_sender.lock().await.send(Message::Text(msg.into())).await;
                }
            }
        });
    }
}
