import { BrowserRouter, Navigate, Route, Routes } from "react-router-dom";
import { AppShell } from "@/components/app-shell";
import { DiagnosticsPage } from "@/pages/diagnostics-page";
import { VisualizerPage } from "@/pages/visualizer-page";
import { GripperPage } from "@/pages/gripper-page";
import { CalibrationPage } from "@/pages/calibration-page";
import { PickPlacePage } from "@/pages/pick-place-page";
import { VisionPage } from "@/pages/vision-page";
import { PlatformBuilderPage } from "@/pages/platform-builder-page";

export default function App() {
  return (
    <BrowserRouter>
      <AppShell>
        <Routes>
          <Route path="/" element={<Navigate to="/diagnostics" replace />} />
          <Route path="/diagnostics" element={<DiagnosticsPage />} />
          <Route path="/visualizer" element={<VisualizerPage />} />
          <Route path="/gripper" element={<GripperPage />} />
          <Route path="/calibration" element={<CalibrationPage />} />
          <Route path="/pick-place" element={<PickPlacePage />} />
          <Route path="/vision" element={<VisionPage />} />
          <Route path="/platform-builder" element={<PlatformBuilderPage />} />
          <Route path="*" element={<Navigate to="/diagnostics" replace />} />
        </Routes>
      </AppShell>
    </BrowserRouter>
  );
}
