import re

filepath = r"c:\ESP32PICO+CAN\_gh_prodrive\index.html"
with open(filepath, "r", encoding="utf-8") as f:
    text = f.read()

# 1. Replace fonts
text = text.replace(
    "family=JetBrains+Mono:wght@400;500;600&family=Orbitron:wght@400;700;900&family=Barlow:wght@300;400;500;600",
    "family=JetBrains+Mono:wght@400;500;600&family=Black+Ops+One&family=Oswald:wght@300;400;500;600"
)
text = text.replace("'Orbitron', sans-serif", "'Black Ops One', system-ui")
text = text.replace("'Barlow', sans-serif", "'Oswald', sans-serif")

# 2. Rename custom CSS properties
text = text.replace("--cyan", "--orange")
text = text.replace("--purple", "--khaki")

# 3. Replace Root block using regex
root_regex = re.compile(r":root\s*\{[^}]+\}", re.MULTILINE)
new_root = """:root {
      --bg: #1e201e;
      --surface: #282a28;
      --border: #444944;
      --text: #e2e8f0;
      --text-dim: #9aa39a;
      --orange: #ff6f00;
      --orange-glow: rgba(255, 111, 0, 0.35);
      --amber: #eab308;
      --amber-glow: rgba(234, 179, 8, 0.3);
      --green: #10b981;
      --green-glow: rgba(16, 185, 129, 0.3);
      --red: #ef4444;
      --red-glow: rgba(239, 68, 68, 0.3);
      --khaki: #d4c4a4;
    }"""
text = root_regex.sub(new_root, text)

# 4. Updates for drawTuneGraph raw JS colors
text = text.replace("'rgba(6, 182, 212, 0.3)'", "'rgba(255, 111, 0, 0.3)'")
text = text.replace("'rgba(6, 182, 212, 0.7)'", "'rgba(255, 111, 0, 0.7)'")
text = text.replace("'#a78bfa'", "'#ff6f00'") # Purlple stroke -> Orange stroke
text = text.replace("'rgba(167, 139, 250, 0.45)'", "'rgba(255, 111, 0, 0.45)'")
text = text.replace("'rgba(167, 139, 250, 0.05)'", "'rgba(255, 111, 0, 0.05)'")
text = text.replace("'#06b6d4'", "'#d4c4a4'") # Node circle stroke Cyan -> Sand

# 5. Overhaul `.card` styling to look rugged
card_old = """.card {
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 1.25rem;
      position: relative;
      overflow: hidden;
      transition: border-color 0.3s;
    }

    .card::before {
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 3px;
      background: var(--orange);
      opacity: 0.5;
    }"""

card_new = """.card {
      background: linear-gradient(145deg, var(--surface), #202220);
      border: 2px solid var(--border);
      border-radius: 4px;
      padding: 1.25rem;
      position: relative;
      overflow: hidden;
      box-shadow: inset 0 0 10px rgba(0,0,0,0.5), 2px 2px 5px rgba(0,0,0,0.3);
      transition: border-color 0.3s;
    }
    
    /* Simulate riveted armor plates at corners */
    .card::after {
      content: '';
      position: absolute; top: 4px; right: 4px; bottom: 4px; left: 4px;
      border: 1px dashed rgba(255,255,255,0.05);
      pointer-events: none;
    }

    .card::before {
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 4px;
      background: repeating-linear-gradient(
        45deg,
        var(--orange),
        var(--orange) 10px,
        #111 10px,
        #111 20px
      );
      opacity: 0.8;
    }"""
text = text.replace(card_old, card_new)

# 6. Change Logo
text = text.replace('<div class="logo">FORJEEPER <span>OBD</span><span class="ver">v1.5</span></div>', '<div class="logo" style="display:flex; align-items:center; gap:0.5rem;"><span style="font-family:monospace; font-weight:bold; letter-spacing:1px; color:var(--text-dim); opacity:0.8; margin-top:-2px;">IIIIIII</span> FORJEEPER <span>OBD</span><span class="ver">v1.5</span></div>')

with open(filepath, "w", encoding="utf-8") as f:
    f.write(text)

print("Jeep tactical styling applied successfully.")
