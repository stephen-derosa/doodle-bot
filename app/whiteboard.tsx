"use client";

import { useEffect, useRef } from "react";

const WIDTH = 1000;
const HEIGHT = 700;

export default function Whiteboard() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const isDrawing = useRef(false);

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
      x: ((e.clientX - rect.left) * WIDTH) / rect.width,
      y: ((e.clientY - rect.top) * HEIGHT) / rect.height,
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
  }

  function handlePointerMove(e: React.PointerEvent<HTMLCanvasElement>) {
    if (!isDrawing.current) return;
    const ctx = e.currentTarget.getContext("2d");
    if (!ctx) return;
    const { x, y } = getPoint(e);
    // Log with the origin at the lower-left corner (y increases upward)
    console.log(`x: ${Math.round(x)}, y: ${Math.round(HEIGHT - y)}`);
    ctx.lineTo(x, y);
    ctx.stroke();
  }

  function handlePointerUp() {
    isDrawing.current = false;
  }

  return (
    <canvas
      ref={canvasRef}
      width={WIDTH}
      height={HEIGHT}
      onPointerDown={handlePointerDown}
      onPointerMove={handlePointerMove}
      onPointerUp={handlePointerUp}
      onPointerCancel={handlePointerUp}
      className="w-full max-w-[1000px] aspect-[10/7] bg-white border border-zinc-300 shadow-sm touch-none cursor-crosshair"
    />
  );
}
