import { useState } from "react";

const modules = [
  {
    id: "core",
    name: "CORE",
    subtitle: "Görev Yöneticisi",
    color: "#DC2626",
    files: ["config.hpp", "mission_controller.hpp/cpp"],
    desc: "Sonlu durum makinesi — 20 farklı uçuş fazı. IDLE → TAKEOFF → SEARCH → TRACK → LOCKON → RTL → LAND tam döngüsü.",
    states: ["IDLE","PRE_ARM","ARMED","TAKEOFF","CLIMB","MISSION_SELECT","SEARCH","TRACK","LOCKON","EVADE","KAMIKAZE_CLIMB","KAMIKAZE_ALIGN","KAMIKAZE_DIVE","KAMIKAZE_PULLUP","LOITER","RTL","LAND","BOUNDARY_RETURN","SIGNAL_LOST","EMERGENCY"]
  },
  {
    id: "vision",
    name: "VISION",
    subtitle: "Görüntü İşleme",
    color: "#2563EB",
    files: ["camera_interface.hpp", "video_pipeline.hpp/cpp", "yolo_detector.hpp/cpp", "target_tracker.hpp/cpp", "lockon_manager.hpp/cpp"],
    desc: "Kamera soyutlama (Gazebo/Gerçek), FFmpeg pipeline, YOLO inference (ONNX/TensorRT), Kalman takip, şartname uyumlu kilitlenme mantığı."
  },
  {
    id: "flight",
    name: "FLIGHT",
    subtitle: "Uçuş Kontrolü",
    color: "#059669",
    files: ["flight_controller.hpp/cpp", "guidance_system.hpp/cpp", "pid_controller.hpp/cpp"],
    desc: "MAVLink/ArduPilot arayüzü, PID tabanlı yaw/pitch güdüm, GPS navigasyon, heading+speed+altitude komutları."
  },
  {
    id: "mission",
    name: "MISSION",
    subtitle: "Görev Modülleri",
    color: "#D97706",
    files: ["kamikaze_module.hpp/cpp", "qr_detector.hpp/cpp", "search_pattern.hpp/cpp", "evasion_module.hpp/cpp"],
    desc: "Kamikaze dalış (≥100m, QR okuma), arama desenleri (spiral/lawnmower/kare), rakip & HSS kaçınma."
  },
  {
    id: "comm",
    name: "COMM",
    subtitle: "Haberleşme",
    color: "#7C3AED",
    files: ["server_comm.hpp/cpp", "telemetry_manager.hpp/cpp"],
    desc: "Hakem sunucusu TCP bağlantısı, telemetri (1-5 Hz), kilitlenme paketi, QR veri iletimi, FTP video yükleme."
  },
  {
    id: "safety",
    name: "SAFETY",
    subtitle: "Güvenlik",
    color: "#E11D48",
    files: ["safety_monitor.hpp/cpp"],
    desc: "Sınır kontrolü (point-in-polygon), irtifa limiti, haberleşme kopması (10sn), batarya, HSS, çarpışma riski."
  },
  {
    id: "recording",
    name: "RECORDING",
    subtitle: "Video Kayıt",
    color: "#0891B2",
    files: ["video_recorder.hpp/cpp (OverlayRenderer dahil)"],
    desc: "FFmpeg pipe encoding (H264/MP4), sunucu saati overlay (ms hassasiyet), kilitlenme dörtgeni (#FF0000, 3px)."
  },
];

const dataFlows = [
  { from: "vision", to: "core", label: "Hedef tespiti, kilitlenme durumu" },
  { from: "core", to: "flight", label: "Güdüm komutları" },
  { from: "core", to: "comm", label: "Telemetri, kilitlenme paketi" },
  { from: "comm", to: "core", label: "Rakip konumlar, sunucu saati" },
  { from: "safety", to: "core", label: "Güvenlik alarmları" },
  { from: "core", to: "mission", label: "Görev tetikleme" },
  { from: "vision", to: "recording", label: "İşlenmiş frame" },
];

const stats = {
  files: 39,
  lines: 5337,
  headers: 17,
  sources: 17,
  configs: 5,
};

export default function App() {
  const [selected, setSelected] = useState(null);
  const selectedModule = modules.find(m => m.id === selected);

  return (
    <div style={{
      minHeight: "100vh",
      background: "#0A0A0F",
      color: "#E5E5E5",
      fontFamily: "'JetBrains Mono', 'Fira Code', monospace",
      padding: "24px",
      overflow: "auto",
    }}>
      {/* Header */}
      <div style={{ textAlign: "center", marginBottom: 32 }}>
        <div style={{
          fontSize: 11,
          letterSpacing: 6,
          color: "#DC2626",
          marginBottom: 8,
          textTransform: "uppercase",
        }}>TEKNOFEST 2026 • SAVAŞAN İHA</div>
        <h1 style={{
          fontSize: 28,
          fontWeight: 700,
          background: "linear-gradient(135deg, #DC2626, #F97316, #EAB308)",
          WebkitBackgroundClip: "text",
          WebkitTextFillColor: "transparent",
          margin: 0,
          letterSpacing: -1,
        }}>SAEROTECH OTONOM SİSTEM MİMARİSİ</h1>
        <div style={{ fontSize: 12, color: "#666", marginTop: 8 }}>
          C++ / ROS2 / OOP / Modüler Tasarım
        </div>
      </div>

      {/* Stats Bar */}
      <div style={{
        display: "flex",
        justifyContent: "center",
        gap: 32,
        marginBottom: 32,
        padding: "12px 0",
        borderTop: "1px solid #1E1E2E",
        borderBottom: "1px solid #1E1E2E",
      }}>
        {Object.entries(stats).map(([key, val]) => (
          <div key={key} style={{ textAlign: "center" }}>
            <div style={{ fontSize: 24, fontWeight: 700, color: "#F97316" }}>{val}</div>
            <div style={{ fontSize: 10, color: "#666", textTransform: "uppercase", letterSpacing: 2 }}>{key}</div>
          </div>
        ))}
      </div>

      {/* Module Grid */}
      <div style={{
        display: "grid",
        gridTemplateColumns: "repeat(auto-fit, minmax(220px, 1fr))",
        gap: 12,
        marginBottom: 24,
      }}>
        {modules.map(m => (
          <button
            key={m.id}
            onClick={() => setSelected(selected === m.id ? null : m.id)}
            style={{
              background: selected === m.id ? `${m.color}15` : "#12121A",
              border: `1px solid ${selected === m.id ? m.color : "#1E1E2E"}`,
              borderRadius: 8,
              padding: "16px",
              cursor: "pointer",
              textAlign: "left",
              transition: "all 0.2s",
              outline: "none",
            }}
          >
            <div style={{
              display: "flex",
              alignItems: "center",
              gap: 8,
              marginBottom: 8,
            }}>
              <div style={{
                width: 8,
                height: 8,
                borderRadius: "50%",
                background: m.color,
                boxShadow: `0 0 8px ${m.color}80`,
              }} />
              <span style={{
                fontSize: 13,
                fontWeight: 700,
                color: m.color,
                letterSpacing: 2,
              }}>{m.name}</span>
            </div>
            <div style={{ fontSize: 12, color: "#999", marginBottom: 4 }}>
              {m.subtitle}
            </div>
            <div style={{ fontSize: 10, color: "#555" }}>
              {m.files.length} dosya
            </div>
          </button>
        ))}
      </div>

      {/* Detail Panel */}
      {selectedModule && (
        <div style={{
          background: "#12121A",
          border: `1px solid ${selectedModule.color}40`,
          borderRadius: 8,
          padding: 20,
          marginBottom: 24,
          animation: "fadeIn 0.2s ease",
        }}>
          <div style={{
            display: "flex",
            alignItems: "center",
            gap: 12,
            marginBottom: 16,
          }}>
            <div style={{
              width: 4,
              height: 40,
              background: selectedModule.color,
              borderRadius: 2,
            }} />
            <div>
              <h2 style={{
                margin: 0,
                fontSize: 18,
                color: selectedModule.color,
              }}>{selectedModule.name} — {selectedModule.subtitle}</h2>
            </div>
          </div>

          <p style={{ fontSize: 13, color: "#BBB", lineHeight: 1.7, margin: "0 0 16px" }}>
            {selectedModule.desc}
          </p>

          <div style={{ marginBottom: 16 }}>
            <div style={{ fontSize: 10, color: "#666", textTransform: "uppercase", letterSpacing: 2, marginBottom: 8 }}>
              Dosyalar
            </div>
            <div style={{ display: "flex", flexWrap: "wrap", gap: 6 }}>
              {selectedModule.files.map(f => (
                <span key={f} style={{
                  fontSize: 11,
                  background: `${selectedModule.color}15`,
                  color: selectedModule.color,
                  padding: "3px 8px",
                  borderRadius: 4,
                  border: `1px solid ${selectedModule.color}30`,
                }}>{f}</span>
              ))}
            </div>
          </div>

          {selectedModule.states && (
            <div>
              <div style={{ fontSize: 10, color: "#666", textTransform: "uppercase", letterSpacing: 2, marginBottom: 8 }}>
                Durum Makinesi Fazları (20)
              </div>
              <div style={{ display: "flex", flexWrap: "wrap", gap: 4 }}>
                {selectedModule.states.map((s, i) => (
                  <span key={s} style={{
                    fontSize: 9,
                    background: "#1A1A24",
                    color: i < 7 ? "#22C55E" : i < 14 ? "#EAB308" : "#EF4444",
                    padding: "2px 6px",
                    borderRadius: 3,
                    border: "1px solid #2A2A34",
                    fontFamily: "monospace",
                  }}>{s}</span>
                ))}
              </div>
            </div>
          )}
        </div>
      )}

      {/* Data Flow */}
      <div style={{
        background: "#12121A",
        border: "1px solid #1E1E2E",
        borderRadius: 8,
        padding: 20,
        marginBottom: 24,
      }}>
        <h3 style={{
          margin: "0 0 16px",
          fontSize: 13,
          color: "#F97316",
          letterSpacing: 2,
          textTransform: "uppercase",
        }}>Veri Akış Haritası</h3>
        <div style={{ display: "grid", gap: 8 }}>
          {dataFlows.map((flow, i) => {
            const from = modules.find(m => m.id === flow.from);
            const to = modules.find(m => m.id === flow.to);
            return (
              <div key={i} style={{
                display: "flex",
                alignItems: "center",
                gap: 8,
                fontSize: 11,
                padding: "6px 0",
                borderBottom: "1px solid #1A1A24",
              }}>
                <span style={{
                  color: from.color,
                  fontWeight: 700,
                  minWidth: 80,
                }}>{from.name}</span>
                <span style={{ color: "#444" }}>→</span>
                <span style={{
                  color: to.color,
                  fontWeight: 700,
                  minWidth: 80,
                }}>{to.name}</span>
                <span style={{ color: "#777", fontSize: 10 }}>{flow.label}</span>
              </div>
            );
          })}
        </div>
      </div>

      {/* Architecture Principles */}
      <div style={{
        background: "#12121A",
        border: "1px solid #1E1E2E",
        borderRadius: 8,
        padding: 20,
      }}>
        <h3 style={{
          margin: "0 0 16px",
          fontSize: 13,
          color: "#22C55E",
          letterSpacing: 2,
          textTransform: "uppercase",
        }}>Tasarım İlkeleri</h3>
        <div style={{
          display: "grid",
          gridTemplateColumns: "repeat(auto-fit, minmax(200px, 1fr))",
          gap: 12,
        }}>
          {[
            { title: "Composition > Inheritance", desc: "Her modül unique_ptr ile sahiplenilir, bağımlılık enjeksiyonu" },
            { title: "Strategy Pattern", desc: "ICameraSource arayüzü — Gazebo veya gerçek kamera aynı API" },
            { title: "Thread-Safe", desc: "Mutex korumalı veri yapıları, atomic değişkenler, lock-free FPS" },
            { title: "Producer-Consumer", desc: "VideoPipeline → YOLO → Tracker → LockonManager pipeline" },
            { title: "State Machine", desc: "20 fazlı FSM, güvenlik kesmeleri her fazı override edebilir" },
            { title: "Şartname Uyumlu", desc: "4sn kilitlenme, %5 kaplama, 1sn tolerans, kara liste" },
          ].map((p, i) => (
            <div key={i} style={{
              background: "#0A0A0F",
              borderRadius: 6,
              padding: 12,
              border: "1px solid #1A1A24",
            }}>
              <div style={{ fontSize: 11, fontWeight: 700, color: "#22C55E", marginBottom: 4 }}>
                {p.title}
              </div>
              <div style={{ fontSize: 10, color: "#777", lineHeight: 1.5 }}>
                {p.desc}
              </div>
            </div>
          ))}
        </div>
      </div>

      <div style={{
        textAlign: "center",
        marginTop: 24,
        fontSize: 10,
        color: "#333",
      }}>
        siha_autonomy • 39 dosya • 5337 satır C++ • ROS2 Jazzy
      </div>
    </div>
  );
}
