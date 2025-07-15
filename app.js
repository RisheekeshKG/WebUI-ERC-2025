// === ROS Connection Setup ===
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

const webStatusEl = document.getElementById('web-status');
const twistStatusEl = document.getElementById('twist-status');
const termEl = document.getElementById('terminal-output');

function updateStatus(el, text, color = '#00ffcc') {
  el.textContent = text;
  el.style.color = color;
}

function logDebug(msg) {
  if (!termEl) return;
  const ts = new Date().toLocaleTimeString();
  termEl.value += `[${ts}] ${msg}\n`;
  termEl.scrollTop = termEl.scrollHeight;
}

ros.on('connection', () => {
  updateStatus(webStatusEl, 'Connected ✅');
  updateStatus(twistStatusEl, 'Idle ⚪', '#ccc');
  logDebug('✅ Connected to rosbridge.');
});

ros.on('error', (e) => {
  updateStatus(webStatusEl, 'Error ❌', '#f55');
  updateStatus(twistStatusEl, 'Error ❌', '#f55');
  logDebug(`❌ ROS error: ${e}`);
});

ros.on('close', () => {
  updateStatus(webStatusEl, 'Disconnected ⚠️', '#fa0');
  updateStatus(twistStatusEl, 'Disconnected ⚠️', '#fa0');
  logDebug('⚠️ Disconnected from rosbridge.');
});

// === Velocity Multipliers ===
let linearMul = 0.5;
let angularMul = 0.5;

const linSlider = document.getElementById('linear-slider');
const angSlider = document.getElementById('angular-slider');

if (linSlider && angSlider) {
  linearMul = parseFloat(linSlider.value);
  angularMul = parseFloat(angSlider.value);

  linSlider.addEventListener('input', (e) => {
    linearMul = parseFloat(e.target.value);
    document.getElementById('linear-val').textContent = linearMul.toFixed(1);
  });

  angSlider.addEventListener('input', (e) => {
    angularMul = parseFloat(e.target.value);
    document.getElementById('angular-val').textContent = angularMul.toFixed(1);
  });
}

// === CmdVel Publisher (TwistStamped) ===
const cmdVel = new ROSLIB.Topic({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/TwistStamped'
});

function getROSTimestamp() {
  const now = Date.now();
  return {
    sec: Math.floor(now / 1000),
    nanosec: (now % 1000) * 1e6
  };
}

function buildTwistStamped(linX, angZ) {
  return new ROSLIB.Message({
    header: {
      frame_id: 'base_link',
      stamp: getROSTimestamp()
    },
    twist: {
      linear: { x: linX, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angZ }
    }
  });
}

function publishTwist(lx, az) {
  cmdVel.publish(buildTwistStamped(lx, az));
}

// === On-screen Joystick (NippleJS) ===
const joystick = nipplejs.create({
  zone: document.getElementById('joystick-container'),
  mode: 'static',
  position: { left: '50%', top: '50%' },
  color: 'cyan',
  size: 180
});

joystick.on('move', (_, data) => {
  if (!data || !data.vector) return;
  const lin = data.vector.y * linearMul;
  const ang = -data.vector.x * angularMul;
  publishTwist(lin, ang);
  updateStatus(twistStatusEl, 'Active 🟢');
  logDebug(`🕹️ Nipple → lin ${lin.toFixed(2)}, ang ${ang.toFixed(2)}`);
});

joystick.on('end', () => {
  publishTwist(0, 0);
  updateStatus(twistStatusEl, 'Idle ⚪', '#ccc');
  logDebug('🛑 Nipple released – stop.');
});

// === Gamepad Control ===
let gamepadIdx = null;

window.addEventListener('gamepadconnected', (e) => {
  gamepadIdx = e.gamepad.index;
  logDebug(`🎮 Gamepad connected: ${e.gamepad.id}`);
  requestAnimationFrame(pollGamepad);
});

window.addEventListener('gamepaddisconnected', () => {
  gamepadIdx = null;
  logDebug('🔌 Gamepad disconnected.');
});

function pollGamepad() {
  const gp = navigator.getGamepads()[gamepadIdx];
  if (gp) {
    const x = gp.axes[0]; // left stick horizontal
    const y = gp.axes[1]; // left stick vertical
    const deadzone = 0.1;
    const lin = Math.abs(y) > deadzone ? -y * linearMul : 0;
    const ang = Math.abs(x) > deadzone ? -x * angularMul : 0;

    publishTwist(lin, ang);
    if (lin || ang) {
      updateStatus(twistStatusEl, 'Active 🟢');
      logDebug(`🎮 Pad → lin ${lin.toFixed(2)}, ang ${ang.toFixed(2)}`);
    } else {
      updateStatus(twistStatusEl, 'Idle ⚪', '#ccc');
    }
  }
  if (gamepadIdx !== null) requestAnimationFrame(pollGamepad);
}

// === LIDAR Canvas Viewer ===
const canvas = document.getElementById('lidar-canvas');
const ctx = canvas?.getContext('2d');

if (ctx) {
  new ROSLIB.Topic({
    ros,
    name: '/scan',
    messageType: 'sensor_msgs/msg/LaserScan'
  }).subscribe((msg) => {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const scale = 40;

    msg.ranges.forEach((r, i) => {
      if (!Number.isFinite(r)) return;
      const a = msg.angle_min + i * msg.angle_increment;
      ctx.beginPath();
      ctx.arc(cx + r * scale * Math.cos(a), cy + r * scale * Math.sin(a), 2, 0, 2 * Math.PI);
      ctx.fillStyle = '#00ffcc';
      ctx.fill();
    });
  });
}

// === Camera Streams ===
const camTopics = {
  cam1: '/image_raw',
  cam2: '/image_raw'
};

Object.entries(camTopics).forEach(([id, topic]) => {
  const img = document.getElementById(id);
  if (!img) return;
  img.src = `http://localhost:8080/stream?topic=${topic}&type=ros_compressed`;
  img.onload = () => {
    img.style.opacity = 1;
    logDebug(`📷 ${id} stream OK.`);
  };
  img.onerror = () => {
    img.style.opacity = 0.2;
    img.alt = 'No stream';
    logDebug(`🚫 ${id} stream fail.`);
  };
});
