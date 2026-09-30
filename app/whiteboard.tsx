"use client";

import { useEffect, useRef, useState } from "react";
import { LocalDataTrack, TokenSource } from "livekit-client";
import { useSession } from "@livekit/components-react";

const encoder = new TextEncoder();

const ROBOT_IDENTITY = 'arm';

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

    session.room.localParticipant.performRpc({
      destinationIdentity: ROBOT_IDENTITY,
      method: 'down',
      payload: '',
    });

    const xInMm = x / PX_PER_MM;
    const yInMm = (HEIGHT_PX - y) / PX_PER_MM;

    const xInMmBounded = Math.min(Math.max(0, xInMm), WIDTH_PX);
    const yInMmBounded = Math.min(Math.max(0, yInMm), HEIGHT_PX);

    console.log(`x ${Math.round(xInMmBounded)}, y: ${Math.round(yInMmBounded)}`);

    const payload = encoder.encode(`${xInMmBounded},${yInMmBounded}`);
    console.log('payload down', payload);
    track?.tryPush({ payload });
  }

  function handlePointerMove(e: React.PointerEvent<HTMLCanvasElement>) {
    if (!isDrawing.current) return;
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) {
      return;
    }

    const { x, y } = getPoint(e);

    ctx.lineTo(x, y);
    ctx.stroke();

    const xInMm = x / PX_PER_MM;
    const yInMm = (HEIGHT_PX - y) / PX_PER_MM;

    const xInMmBounded = Math.min(Math.max(0, xInMm), WIDTH_PX);
    const yInMmBounded = Math.min(Math.max(0, yInMm), HEIGHT_PX);

    console.log(`x ${Math.round(xInMmBounded)}, y: ${Math.round(yInMmBounded)}`);

    const payload = encoder.encode(`${xInMmBounded},${yInMmBounded}`);
    console.log('payload', payload);
    track?.tryPush({ payload });
  }

  function handlePointerUp() {
    isDrawing.current = false;
  }

  return (
    <canvas
      ref={canvasRef}
      width={WIDTH_PX}
      height={HEIGHT_PX}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
      className="w-full bg-white border border-zinc-300 shadow-sm touch-none cursor-crosshair"
      style={{ width: WIDTH_PX, height: HEIGHT_PX }}
    />
  );
}
