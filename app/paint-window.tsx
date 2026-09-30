import type { ReactNode, Ref } from "react";

// The classic 28-color MS Paint palette, top row then bottom row.
const PALETTE = [
  "#000000", "#808080", "#800000", "#808000", "#008000", "#008080", "#000080",
  "#800080", "#808040", "#004040", "#0080ff", "#004080", "#8000ff", "#804000",
  "#ffffff", "#c0c0c0", "#ff0000", "#ffff00", "#00ff00", "#00ffff", "#0000ff",
  "#ff00ff", "#ffff80", "#00ff80", "#80ffff", "#8080ff", "#ff0080", "#ff8040",
];

const MENUS = ["File", "Edit", "View", "Image", "Colors", "Help"];

// 16x16 pixel-art-ish icons for the toolbox. Only the pencil is "real".
const TOOLS: { name: string; icon: ReactNode }[] = [
  {
    name: "Free-Form Select",
    icon: <path d="M3 4 L8 2 L13 5 L12 11 L6 13 L2 9 Z" fill="none" stroke="#000" strokeDasharray="1 1" />,
  },
  {
    name: "Select",
    icon: <rect x="2.5" y="3.5" width="11" height="9" fill="none" stroke="#000" strokeDasharray="1 1" />,
  },
  {
    name: "Eraser/Color Eraser",
    icon: (
      <>
        <path d="M3 10 L8 5 L13 8 L8 13 Z" fill="#ffff80" stroke="#000" />
        <path d="M3 10 L8 13 L8 14 L3 11 Z" fill="#c0c0c0" stroke="#000" />
      </>
    ),
  },
  {
    name: "Fill With Color",
    icon: (
      <>
        <path d="M4 7 L8 3 L13 8 L9 12 Z" fill="#fff" stroke="#000" />
        <path d="M13 8 L14 12 L13 13" fill="none" stroke="#000080" strokeWidth="1.5" />
      </>
    ),
  },
  {
    name: "Pick Color",
    icon: <path d="M3 13 L10 6 M9 4 L12 7 M11 3 L13 5" stroke="#000" strokeWidth="1.5" />,
  },
  {
    name: "Magnifier",
    icon: (
      <>
        <circle cx="7" cy="7" r="4" fill="#80ffff" stroke="#000" />
        <path d="M10 10 L14 14" stroke="#000" strokeWidth="2" />
      </>
    ),
  },
  {
    name: "Pencil",
    icon: (
      <>
        <path d="M3 13 L4 10 L11 3 L13 5 L6 12 Z" fill="#ffff00" stroke="#000" />
        <path d="M3 13 L4 10 L6 12 Z" fill="#000" />
      </>
    ),
  },
  {
    name: "Brush",
    icon: (
      <>
        <path d="M9 7 L13 2 L14 3 L10 8" fill="#804000" stroke="#000" />
        <path d="M3 13 C4 9 7 8 9 9 C9 12 6 13 3 13 Z" fill="#000080" />
      </>
    ),
  },
  {
    name: "Airbrush",
    icon: (
      <>
        <rect x="8" y="6" width="5" height="8" fill="#c0c0c0" stroke="#000" />
        <path d="M2 4h1M4 2h1M3 6h1M5 5h1M1 7h1M6 3h1" stroke="#000" />
      </>
    ),
  },
  {
    name: "Text",
    icon: <text x="3" y="13" fontSize="13" fontFamily="serif" fontWeight="bold">A</text>,
  },
  { name: "Line", icon: <path d="M2 13 L14 3" stroke="#000" strokeWidth="1.5" /> },
  { name: "Curve", icon: <path d="M2 12 C6 0 10 16 14 4" fill="none" stroke="#000" strokeWidth="1.5" /> },
  { name: "Rectangle", icon: <rect x="2.5" y="4.5" width="11" height="8" fill="none" stroke="#000" /> },
  { name: "Polygon", icon: <path d="M3 13 L3 6 L8 3 L13 7 L10 13 Z" fill="none" stroke="#000" /> },
  { name: "Ellipse", icon: <ellipse cx="8" cy="8.5" rx="6" ry="4.5" fill="none" stroke="#000" /> },
  { name: "Rounded Rectangle", icon: <rect x="2.5" y="4.5" width="11" height="8" rx="3" fill="none" stroke="#000" /> },
];

const ACTIVE_TOOL = "Pencil";

export default function PaintWindow({
  children,
  coordsRef,
  sizeLabel,
}: {
  children: ReactNode;
  coordsRef: Ref<HTMLSpanElement>;
  sizeLabel: string;
}) {
  return (
    <div className="win98 win98-raised flex h-full max-h-[calc(100vh-4rem)] w-full max-w-[900px] flex-col p-[3px]">
      <div className="win98-titlebar">
        <div className="flex items-center gap-1">
          <PaintIcon />
          <span>untitled - Paint</span>
        </div>
        <div className="flex gap-[2px]">
          <button className="win98-titlebutton" aria-label="Minimize">
            <svg width="8" height="7"><rect x="0" y="5" width="6" height="2" /></svg>
          </button>
          <button className="win98-titlebutton" aria-label="Maximize">
            <svg width="9" height="9"><path d="M0.5 0.5h8v8h-8z M0.5 1.5h8" fill="none" stroke="#000" /></svg>
          </button>
          <button className="win98-titlebutton ml-[2px]" aria-label="Close">
            <svg width="8" height="7"><path d="M0 0l8 7M8 0l-8 7" stroke="#000" strokeWidth="1.5" /></svg>
          </button>
        </div>
      </div>

      <div className="flex gap-[2px] px-[1px] py-[1px]">
        {MENUS.map((m) => (
          <button key={m} className="win98-menuitem">
            <u>{m[0]}</u>
            {m.slice(1)}
          </button>
        ))}
      </div>

      <div className="flex min-h-0 flex-1 gap-[2px]">
        <div className="flex w-[56px] shrink-0 flex-col items-center gap-2 pt-[2px]">
          <div className="grid grid-cols-2">
            {TOOLS.map((t) => (
              <button
                key={t.name}
                title={t.name}
                className={`win98-tool ${t.name === ACTIVE_TOOL ? "win98-tool-active" : ""}`}
              >
                <svg width="16" height="16" viewBox="0 0 16 16" shapeRendering="crispEdges">
                  {t.icon}
                </svg>
              </button>
            ))}
          </div>
          <div className="win98-sunken flex h-[66px] w-[42px] flex-col items-center justify-around bg-[#c0c0c0]">
            {[1, 2, 3, 4, 5].map((w) => (
              <div
                key={w}
                className={`w-[28px] ${w === 2 ? "bg-[#000080]" : ""} flex h-[9px] items-center justify-center`}
              >
                <div className={w === 2 ? "bg-white" : "bg-black"} style={{ height: w, width: 24 }} />
              </div>
            ))}
          </div>
        </div>

        <div className="win98-canvas-area min-w-0 flex-1 overflow-auto">
          <div className="relative m-[3px] inline-block">
            {children}
            <span className="win98-handle" style={{ right: -4, bottom: -4 }} />
            <span className="win98-handle" style={{ right: -4, top: "50%" }} />
            <span className="win98-handle" style={{ left: "50%", bottom: -4 }} />
          </div>
        </div>
      </div>

      <div className="flex items-center gap-1 py-[3px]">
        <div className="win98-sunken relative h-[31px] w-[31px] shrink-0 bg-[#c0c0c0]">
          <div className="win98-sunken absolute right-[4px] bottom-[4px] h-[14px] w-[14px] bg-white" />
          <div className="win98-sunken absolute top-[4px] left-[4px] h-[14px] w-[14px] bg-black" />
        </div>
        <div className="grid grid-flow-col grid-rows-2 gap-0">
          {PALETTE.slice(0, 14).map((c, i) => (
            <Swatch key={c} top={c} bottom={PALETTE[i + 14]} />
          ))}
        </div>
      </div>

      <div className="flex gap-[2px]">
        <div className="win98-status flex-1">For Help, click Help Topics on the Help Menu.</div>
        <div className="win98-status w-[110px]">
          <span ref={coordsRef} />
        </div>
        <div className="win98-status w-[110px]">{sizeLabel}</div>
      </div>
    </div>
  );
}

function Swatch({ top, bottom }: { top: string; bottom: string }) {
  return (
    <>
      <div className="win98-sunken h-[16px] w-[16px]" style={{ background: top }} />
      <div className="win98-sunken h-[16px] w-[16px]" style={{ background: bottom }} />
    </>
  );
}

function PaintIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" shapeRendering="crispEdges">
      <rect x="1" y="3" width="11" height="11" fill="#fff" stroke="#000" />
      <rect x="3" y="5" width="3" height="3" fill="#ff0000" />
      <rect x="7" y="7" width="3" height="3" fill="#0000ff" />
      <rect x="3" y="9" width="3" height="3" fill="#ffff00" />
      <path d="M9 9 L15 1" stroke="#804000" strokeWidth="2" />
    </svg>
  );
}
