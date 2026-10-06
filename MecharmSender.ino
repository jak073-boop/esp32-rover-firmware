#include <esp_now.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#define XPIN  34  //analog
#define YPIN  35  //analog
#define SWPIN 14  //digital
#define X2PIN 33
#define Y2PIN 32
#define SW2PIN 13

#define soilbuttonpin 21 // button
#define BMEbuttonpin 15

#define soilledpin 23
#define BMEledpin 18

// MAC address of receiver ESP32
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x80, 0x3F, 0x54};  // 

typedef struct struct_message {
  int xVal;
  int yVal;
  int zVal;

  int xVal2;
  int yVal2;
  int zVal2;

  int soilstate;
  int BMEstate;
} struct_message;

struct_message jsData;

esp_now_peer_info_t peerInfo;

const char *ssid     = "RoverAP";
const char *password = "rover1234";

AsyncWebServer server(80);
AsyncEventSource events("/events");

typedef struct {
  float     tempF;
  float     rh;
  float     pressure_hPa;
  int       soil_pct;
  bool      BME_on;
  bool      soil_on;
} rover_to_controller;

volatile rover_to_controller rxTelem;
volatile bool telemUpdated = false;

String processor(const String& var) {
  if (var == "TEMPERATURE") return String(rxTelem.tempF, 2);
  if (var == "HUMIDITY")    return String(rxTelem.rh, 2);
  if (var == "PRESSURE")    return String(rxTelem.pressure_hPa, 2);
  if (var == "SOIL")        return String(rxTelem.soil_pct);
  return String();
}

// html/css
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE HTML><html lang="en">
<head>
  <meta charset="utf-8">
  <title>JAJA MECH Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root { --ink:#1b1f24; --muted:#6b7a8c; --brand:#1e3a8a;
            --card:#ffffff; --line:#e6ebf1; --off:#d9534f; --on:#2bb673; }
    *{box-sizing:border-box} html,body{margin:0}
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;background:#f4f6f9;color:var(--ink)}
    .topnav{background:var(--brand);color:#fff;padding:14px 18px}
    .topnav h3{margin:0;font-weight:700;letter-spacing:.3px}
    .content{padding:20px}
    .cards{max-width:1100px;margin:0 auto;display:grid;grid-gap:18px;
           grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}
    .card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px 18px}
    .card h4{margin:0 0 8px 0;font-size:.95rem;color:var(--muted);letter-spacing:.2px;text-transform:uppercase}
    .reading{font-size:2rem;font-weight:700;line-height:1}
    .unit{font-size:1rem;color:var(--muted);margin-left:6px}
    .row{display:flex;align-items:baseline;gap:6px}
    .pill{display:inline-block;border:1px solid var(--line);border-radius:999px;padding:2px 8px;font-size:.85rem;color:var(--muted)}
    .pill.on{color:var(--on);border-color:rgba(43,182,115,.35)}
    .pill.off{color:var(--off);border-color:rgba(217,83,79,.35)}
    .off .reading,.off .unit{visibility:hidden}
    .off .off-tag{display:inline-block}
    .off-tag{display:none;font-weight:700;color:var(--muted)}
    .spark{width:100%; height:60px; display:block; margin-top:8px}
    .tip{position:fixed;pointer-events:none;background:#111;color:#fff;font-size:.8rem;
         padding:6px 8px;border-radius:6px;box-shadow:0 4px 12px rgba(0,0,0,.18);display:none}
  </style>
</head>
<body>
  <div class="topnav"><h3>Sensor Readings</h3></div>
  <div class="content">
    <div class="cards">

      <div class="card temperature" id="card-temp">
        <h4>TEMPERATURE <span class="pill" id="pill-bme">BME: unknown</span></h4>
        <div class="row">
          <span class="reading" id="temp">--</span><span class="unit">&deg;F</span>
          <span class="off-tag">OFF</span>
        </div>
        <canvas class="spark" id="spark-temp" width="600" height="60"></canvas>
      </div>

      <div class="card humidity" id="card-hum">
        <h4>HUMIDITY <span class="pill" id="pill-bme-2">BME: unknown</span></h4>
        <div class="row">
          <span class="reading" id="hum">--</span><span class="unit">&percnt;</span>
          <span class="off-tag">OFF</span>
        </div>
        <canvas class="spark" id="spark-hum" width="600" height="60"></canvas>
      </div>

      <div class="card pressure" id="card-pres">
        <h4>PRESSURE <span class="pill" id="pill-bme-3">BME: unknown</span></h4>
        <div class="row">
          <span class="reading" id="pres">--</span><span class="unit">hPa</span>
          <span class="off-tag">OFF</span>
        </div>
        <canvas class="spark" id="spark-pres" width="600" height="60"></canvas>
      </div>

      <div class="card soil" id="card-soil">
        <h4>SOIL <span class="pill" id="pill-soil">SOIL: unknown</span></h4>
        <div class="row">
          <span class="reading" id="soil">--</span><span class="unit">&percnt;</span>
          <span class="off-tag">OFF</span>
        </div>
        <canvas class="spark" id="spark-soil" width="600" height="60"></canvas>
      </div>

    </div>
  </div>

<script>
  <!-- DOM HELPERS -->
  const setTxt=(id,v)=>{const e=document.getElementById(id); if(e) e.textContent=v};
  const setOnOff=(on, cardIds, pillIds, label)=>{
    cardIds.forEach(cid=>{const c=document.getElementById(cid); if(c) c.classList.toggle('off', !on);});
    pillIds.forEach(pid=>{
      const p=document.getElementById(pid); if(!p) return;
      p.textContent = label + (on ? ' ON' : ' OFF');
      p.classList.toggle('on', on); p.classList.toggle('off', !on);
    });
  };

  <!-- Series -->
  const MAX_SAMPLES = 240;
  function makeSeries(ymin=null, ymax=null, fixed=2, unit=''){
    return {
      y: [], t: [], ymin, ymax, fixed, unit,
      on: null, elapsed: 0, lastOnAt: 0,
      push(v){
        if(!Number.isFinite(v)) return;
        const now = performance.now();
        const active = this.elapsed + (this.on ? (now - this.lastOnAt) : 0);
        this.y.push(v); this.t.push(active);
        if(this.y.length > MAX_SAMPLES){ this.y.shift(); this.t.shift(); }
      }
    };
  }

  const series = {
    temp:     makeSeries(null, null, 2, '°F'),
    humidity: makeSeries(null, null, 2, '%'),
    pres:     makeSeries(null, null, 2, 'hPa'),
    soilpct:  makeSeries(0, 100, 0, '%'),
  };

  // Per-canvas state: store last draw math so hover uses identical mapping.
  const canvState = {
    'spark-temp': {key:'temp',     W:600, H:60, dx:0, ymin:null, ymax:null, hoverIdx:null},
    'spark-hum' : {key:'humidity', W:600, H:60, dx:0, ymin:null, ymax:null, hoverIdx:null},
    'spark-pres': {key:'pres',     W:600, H:60, dx:0, ymin:null, ymax:null, hoverIdx:null},
    'spark-soil': {key:'soilpct',  W:600, H:60, dx:0, ymin:null, ymax:null, hoverIdx:null},
  };

  <!-- drawing -->
  function drawSpark(canvasId, s){
    const c = document.getElementById(canvasId);
    if(!c || !c.getContext) return;
    const ctx = c.getContext('2d');
    const W = c.width, H = c.height;
    ctx.clearRect(0,0,W,H);
    if(!s || s.y.length < 2) return;

    let ymin = (s.ymin !== null) ? s.ymin : Math.min(...s.y);
    let ymax = (s.ymax !== null) ? s.ymax : Math.max(...s.y);
    if (ymax === ymin){ ymax += 1; ymin -= 1; }
    const pad = (ymax - ymin) * 0.08;
    ymin -= pad; ymax += pad;

    const n = s.y.length;
    const dx = W / (n - 1);
    const st = canvState[canvasId];
    st.dx = dx; st.ymin = ymin; st.ymax = ymax;

    <!-- baseline -->
    ctx.globalAlpha = 0.2;
    ctx.beginPath(); ctx.moveTo(0, H-0.5); ctx.lineTo(W, H-0.5); ctx.stroke();
    ctx.globalAlpha = 1;

    <!-- line -->
    ctx.beginPath();
    for(let i=0;i<n;i++){
      const x = i * dx;
      const y = H - ((s.y[i]-ymin) / (ymax - ymin)) * H;
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.lineWidth = 2; ctx.stroke();

    <!-- last point dot -->
    const last = s.y[n-1];
    const lx = W; const ly = H - ((last - ymin) / (ymax - ymin)) * H;
    ctx.beginPath(); ctx.arc(lx, ly, 3, 0, Math.PI*2); ctx.fill();

    <!-- hover marker (if any) -->
    if (st.hoverIdx !== null && st.hoverIdx >= 0 && st.hoverIdx < n){
      const hx = st.hoverIdx * dx;
      const hy = H - ((s.y[st.hoverIdx]-ymin) / (ymax - ymin)) * H;

      ctx.save();
      ctx.globalAlpha = 0.25;
      ctx.beginPath(); ctx.moveTo(hx+0.5, 0); ctx.lineTo(hx+0.5, H); ctx.stroke();
      ctx.globalAlpha = 1;
      ctx.beginPath(); ctx.arc(hx, hy, 4, 0, Math.PI*2); ctx.fill();
      ctx.restore();
    }
  }

  <!-- hover tooltip  -->
  const tip = document.createElement('div');
  tip.className = 'tip';
  document.body.appendChild(tip);
  const msToHMS = (ms)=>{let s=Math.max(0,Math.floor(ms/1000));const h=Math.floor(s/3600);s-=h*3600;const m=Math.floor(s/60);s-=m*60;const pad=n=>n.toString().padStart(2,'0');return `${pad(h)}:${pad(m)}:${pad(s)}`;};

  function screenY(val, H, ymin, ymax){
    return H - ((val - ymin) / (ymax - ymin)) * H;
  }

  function attachHover(canvasId){
    const c = document.getElementById(canvasId); if(!c) return;
    const st = canvState[canvasId];
    const key = st.key;

    c.addEventListener('mousemove', (ev)=>{
      const rect = c.getBoundingClientRect();
      const scaleX = c.width / rect.width;
      const px = (ev.clientX - rect.left) * scaleX;

      const s = series[key]; const n = s.y.length;
      if (n < 2 || st.dx === 0 || st.ymin === null){ tip.style.display='none'; return; }

      // initial x-based estimate
      let i = Math.round(px / st.dx);
      i = Math.max(0, Math.min(n-1, i));

      <!-- refine: check ±2 around i for smallest 2D distance to cursor -->
      const H = c.height;
      const ymin = st.ymin, ymax = st.ymax;
      let best = i, bestD = Infinity;
      for (let j = Math.max(0, i-2); j <= Math.min(n-1, i+2); j++){
        const xj = j * st.dx;
        const yj = screenY(s.y[j], H, ymin, ymax);
        const dx = px - xj;
        const dy = (ev.clientY - rect.top) * (H/rect.height) - yj; // scale Y too
        const d2 = dx*dx + dy*dy;
        if (d2 < bestD){ bestD = d2; best = j; }
      }
      st.hoverIdx = best;

      const val = s.y[best], tms = s.t[best];
      tip.textContent = `${val.toFixed(s.fixed)} ${s.unit} @ ${msToHMS(tms)}`;
      tip.style.left = (ev.clientX + 12) + 'px';
      tip.style.top  = (ev.clientY + 12) + 'px';
      tip.style.display = 'block';

      drawSpark(canvasId, s);
    });

    c.addEventListener('mouseleave', ()=>{
      st.hoverIdx = null;
      tip.style.display='none';
      drawSpark(canvasId, series[st.key]); // redraw to clear marker
    });
  }
  ['spark-temp','spark-hum','spark-pres','spark-soil'].forEach(attachHover);

  function syncCanvasSize(id) {
    const c = document.getElementById(id);
    if (!c) return;
    const rect = c.getBoundingClientRect();
    const w = Math.max(2, Math.round(rect.width));
    const h = Math.max(2, Math.round(rect.height));
    if (c.width !== w || c.height !== h) {
      c.width = w; c.height = h;
      const st = canvState[id]; if (st){ st.W = w; st.H = h; }
      const key = canvState[id]?.key;
      if (key) drawSpark(id, series[key]);   // redraw after resize
    }
  }
  function syncAllCanvases(){
    Object.keys(canvState).forEach(syncCanvasSize);
  }
  window.addEventListener('load', syncAllCanvases);
  window.addEventListener('resize', ()=>{
    clearTimeout(window.__sparkResizeT);
    window.__sparkResizeT = setTimeout(syncAllCanvases, 100);
  });

  let bmeOn=null, soilOn=null;
  function setModuleOn(keys, on){
    keys.forEach(k=>{
      const s=series[k]; if(s.on===on) return;
      const now=performance.now();
      if(on){ s.lastOnAt=now; s.on=true; }
      else { if(s.on){ s.elapsed += (now - s.lastOnAt); } s.on=false; }
    });
  }

  if(!!window.EventSource){
    const es=new EventSource('/events');

    es.addEventListener('bme_on', e=>{
      bmeOn = (e.data.trim()==='1');
      setOnOff(bmeOn, ['card-temp','card-hum','card-pres'], ['pill-bme','pill-bme-2','pill-bme-3'], 'BME:');
      setModuleOn(['temp','humidity','pres'], bmeOn);
    });

    es.addEventListener('soil_on', e=>{
      soilOn = (e.data.trim()==='1');
      setOnOff(soilOn, ['card-soil'], ['pill-soil'], 'SOIL:');
      setModuleOn(['soilpct'], soilOn);
    });

    es.addEventListener('temperature', e=>{
      if(bmeOn===false) return;
      const v=parseFloat(e.data);
      if(Number.isFinite(v)){
        setTxt('temp', v.toFixed(series.temp.fixed));
        series.temp.push(v);
        drawSpark('spark-temp', series.temp);
      }
    });

    es.addEventListener('humidity', e=>{
      if(bmeOn===false) return;
      const v=parseFloat(e.data);
      if(Number.isFinite(v)){
        setTxt('hum', v.toFixed(series.humidity.fixed));
        series.humidity.push(v);
        drawSpark('spark-hum', series.humidity);
      }
    });

    es.addEventListener('pressure', e=>{
      if(bmeOn===false) return;
      const v=parseFloat(e.data);
      if(Number.isFinite(v)){
        setTxt('pres', v.toFixed(series.pres.fixed));
        series.pres.push(v);
        drawSpark('spark-pres', series.pres);
      }
    });

    es.addEventListener('soil', e=>{
      if(soilOn===false) return;
      const v=parseFloat(e.data);
      if(Number.isFinite(v)){
        setTxt('soil', v.toFixed(series.soilpct.fixed));
        series.soilpct.push(v);
        drawSpark('spark-soil', series.soilpct);
      }
    });
  }
</script>
</body>
</html>
)HTML";


// ESP-NOW callbacks
void OnDataSent(const esp_now_send_info_t *mac_addr, esp_now_send_status_t status) {
  // Optional debug
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(rover_to_controller)) {
    memcpy((void*)&rxTelem, data, sizeof(rxTelem));
    telemUpdated = true;

    if (rxTelem.BME_on) {
      events.send("1", "bme_on", millis());
      events.send(String(rxTelem.tempF, 2).c_str(), "temperature", millis());
      events.send(String(rxTelem.rh, 2).c_str(), "humidity",    millis());
      events.send(String(rxTelem.pressure_hPa, 2).c_str(), "pressure", millis());
    } else {
      events.send("0", "bme_on", millis());
    }

    if (rxTelem.soil_on) {
      events.send("1", "soil_on", millis());
      events.send(String(rxTelem.soil_pct).c_str(), "soil", millis());
    } else {
      events.send("0", "soil_on", millis());
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(SWPIN,  INPUT_PULLUP);
  pinMode(SW2PIN, INPUT_PULLUP);
  pinMode(soilbuttonpin, INPUT_PULLUP);
  pinMode(BMEbuttonpin,  INPUT_PULLUP);
  pinMode(soilledpin, OUTPUT);
  pinMode(BMEledpin, OUTPUT);

  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  WiFi.softAP(ssid, password);
  Serial.print("Connect to access point: "); Serial.println(ssid);
  Serial.println(String("Soft-AP IP address = ") + WiFi.softAPIP().toString());
  delay(100);

  // HTTP routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  // SSE handler (also pushes current states so pills don't stay "unknown")
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected. Last ID: %u\n", client->lastId());
    }
    client->send("hello!", NULL, millis(), 10000);
    // Initialize pill states immediately
    client->send(rxTelem.BME_on  ? "1" : "0", "bme_on",  millis());
    client->send(rxTelem.soil_on ? "1" : "0", "soil_on", millis());
  });
  server.addHandler(&events);
  server.begin();
}

void loop() {
  static bool soilstate = false;
  static bool BMEstate  = false;

  static bool soilLED = false;
  static bool soilheld = false;

  static bool BMELED = false;
  static bool BMEheld = false;

  // Debounced button toggles
  if (digitalRead(soilbuttonpin) == LOW && soilheld == false) {
    delay(20);
    if (digitalRead(soilbuttonpin) == LOW) {
      soilLED = !soilLED;
      digitalWrite(soilledpin, soilLED ? HIGH : LOW);
      soilstate = soilLED;
      soilheld = true;
    }
  } else if (digitalRead(soilbuttonpin) == HIGH && soilheld == true) {
    delay(10);
    soilheld = false;
  }

  if (digitalRead(BMEbuttonpin) == LOW && BMEheld == false) {
    delay(20);
    if (digitalRead(BMEbuttonpin) == LOW) {
      BMELED = !BMELED;
      digitalWrite(BMEledpin, BMELED ? HIGH : LOW);
      BMEstate = BMELED;
      BMEheld = true;
    }
  } else if (digitalRead(BMEbuttonpin) == HIGH && BMEheld == true) {
    delay(10);
    BMEheld = false;
  }

  // every 500 ms
  static unsigned long lastSentTime = 0;
  if (millis() - lastSentTime >= 500) {
    lastSentTime = millis();

    jsData.xVal  = analogRead(XPIN);
    jsData.yVal  = analogRead(YPIN);
    jsData.zVal  = digitalRead(SWPIN);

    jsData.xVal2 = analogRead(X2PIN);
    jsData.yVal2 = analogRead(Y2PIN);
    jsData.zVal2 = digitalRead(SW2PIN);

    jsData.soilstate = soilstate;
    jsData.BMEstate  = BMEstate;

    (void)esp_now_send(broadcastAddress, (uint8_t *)&jsData, sizeof(jsData));
  }

  static unsigned long plot_t = 0;
  if (millis() - plot_t >= 500) {
    plot_t = millis();
    if (telemUpdated == true) {
      telemUpdated = false;
      if (rxTelem.BME_on && rxTelem.soil_on) {
        Serial.print("humidity:");      Serial.print(rxTelem.rh);
        Serial.print(" pressure:");     Serial.print(rxTelem.pressure_hPa);
        Serial.print(" soilmoisture:"); Serial.print(rxTelem.soil_pct); Serial.print("%");
        Serial.print(" temperatureF:"); Serial.println(rxTelem.tempF);
      } else if (rxTelem.BME_on) {
        Serial.print("humidity:");      Serial.print(rxTelem.rh);
        Serial.print(" pressure:");     Serial.print(rxTelem.pressure_hPa);
        Serial.print(" temperatureF:"); Serial.println(rxTelem.tempF);
      } else if (rxTelem.soil_on) {
        Serial.print("soilmoisture:");  Serial.print(rxTelem.soil_pct); Serial.println("%");
      }
    }
  }
}
