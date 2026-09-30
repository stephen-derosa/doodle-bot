"use client";

import { useEffect, useRef, useState } from "react";
import { LocalDataTrack, TokenSource } from "livekit-client";
import { useSession } from "@livekit/components-react";
import PaintWindow from "./paint-window";

const encoder = new TextEncoder();

// Third coord in each "<x>,<y>,<z>" payload. Swap if the arm reads z the other way.
const PEN_UP_Z = 1;
const PEN_DOWN_Z = 0;

const PX_PER_MM = 5;

const WIDTH_MM = 100;
const HEIGHT_MM = 150;

const WIDTH_PX = WIDTH_MM * PX_PER_MM;
const HEIGHT_PX = HEIGHT_MM * PX_PER_MM;

const tokenSource = TokenSource.literal({
  serverUrl: "wss://ryan-test-e2nv0bdm.livekit.cloud",
  participantToken: "eyJhbGciOiJIUzI1NiJ9.eyJ2aWRlbyI6eyJyb29tIjoicm9vbW5hbWUxMjMiLCJyb29tSm9pbiI6dHJ1ZSwicm9vbUNyZWF0ZSI6dHJ1ZSwiY2FuUHVibGlzaCI6dHJ1ZSwiY2FuUHVibGlzaERhdGEiOnRydWUsImNhblVwZGF0ZU93bk1ldGFkYXRhIjp0cnVlfSwicm9vbUNvbmZpZyI6eyJuYW1lIjoiIiwiZW1wdHlUaW1lb3V0IjowLCJkZXBhcnR1cmVUaW1lb3V0IjowLCJtYXhQYXJ0aWNpcGFudHMiOjAsIm1pblBsYXlvdXREZWxheSI6MCwibWF4UGxheW91dERlbGF5IjowLCJzeW5jU3RyZWFtcyI6ZmFsc2UsImFnZW50cyI6W3siYWdlbnROYW1lIjoibXktYWdlbnQtanMiLCJtZXRhZGF0YSI6IiIsInJlc3RhcnRQb2xpY3kiOiJKUlBfT05fRkFJTFVSRSIsImRlcGxveW1lbnQiOiIiLCJhdHRyaWJ1dGVzIjp7fX1dLCJtZXRhZGF0YSI6IiIsInRhZ3MiOnt9fSwiaXNzIjoiQVBJdlZSUWJMV0RkUG9OIiwiZXhwIjoxNzkwODEwNzY4LCJuYmYiOjAsInN1YiI6ImV4YW1wbGUtcGFydGljaXBhbnQtMDk5NzQ2MzY3MzE2MDc5MDgifQ.Q9AvnYW-xmHCtQCNtWjWdNshjF69ULikt6a4W_jOp8A",
});

export default function Whiteboard() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const isDrawing = useRef(false);
  const coordsRef = useRef<HTMLSpanElement>(null);
  const lastPoint = useRef({ x: 0, y: 0 });

  const session = useSession(tokenSource);

  const [track, setTrack] = useState<LocalDataTrack | null>(null);

  useEffect(() => {
    session.start().then(async () => {
      const track = await session.room.localParticipant.publishDataTrack({ name: 'coordinates' });
      setTrack(track);
    });
    return () => {
      session.end();
    };
  }, []);

  console.log('SESSION', session);

  useEffect(() => {
    const ctx = canvasRef.current?.getContext("2d");
    if (!ctx) return;
    ctx.strokeStyle = "black";
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";
  }, []);

  function getPoint(e: React.PointerEvent<HTMLCanvasElement>) {
    const rect = e.currentTarget.getBoundingClientRect();
    return {
      x: ((e.clientX - rect.left) * WIDTH_PX) / rect.width,
      y: (((e.clientY - rect.top) * HEIGHT_PX) / rect.height),
    };
  }

  function handlePointerDown(e: React.PointerEvent<HTMLCanvasElement>) {
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) return;
    e.currentTarget.setPointerCapture(e.pointerId);
    isDrawing.current = true;
    const { x, y } = getPoint(e);
    ctx.beginPath();
    ctx.moveTo(x, y);

    // Travel to the stroke start with the pen up, then lower it.
    push(x, y, PEN_UP_Z);
    push(x, y, PEN_DOWN_Z);
  }

  function handlePointerMove(e: React.PointerEvent<HTMLCanvasElement>) {
    showCoords(e);
    if (!isDrawing.current) return;
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) {
      return;
    }

    const { x, y } = getPoint(e);

    ctx.lineTo(x, y);
    ctx.stroke();

    push(x, y, PEN_DOWN_Z);
  }

  function handlePointerUp() {
    if (!isDrawing.current) return;
    isDrawing.current = false;
    push(lastPoint.current.x, lastPoint.current.y, PEN_UP_Z);
  }

  function push(x: number, y: number, z: number) {
    lastPoint.current = { x, y };
    const xInMm = Math.min(Math.max(0, x / PX_PER_MM), WIDTH_MM);
    const yInMm = Math.min(Math.max(0, (HEIGHT_PX - y) / PX_PER_MM), HEIGHT_MM);
    track?.tryPush({ payload: encoder.encode(`${xInMm},${yInMm},${z}`) });
  }

  // Written straight to the DOM so hovering doesn't re-render the component.
  function showCoords(e: React.PointerEvent<HTMLCanvasElement>) {
    if (!coordsRef.current) return;
    const { x, y } = getPoint(e);
    coordsRef.current.textContent = `${Math.round(x)},${Math.round(y)}`;
  }

  function clearCoords() {
    if (coordsRef.current) coordsRef.current.textContent = "";
  }

  return (
    <PaintWindow coordsRef={coordsRef} sizeLabel={`${WIDTH_PX}x${HEIGHT_PX}`}>
      <canvas
        ref={canvasRef}
        width={WIDTH_PX}
        height={HEIGHT_PX}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp}
        onPointerCancel={handlePointerUp}
        onPointerLeave={clearCoords}
        className="block bg-white touch-none cursor-crosshair"
        style={{ width: WIDTH_PX, height: HEIGHT_PX }}
      />
    </PaintWindow>
  );
}
