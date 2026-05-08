#region Usings
using System;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
#endregion

public class Script_Instance : GH_ScriptInstance
{
  private void RunScript(
		Brep B,
		Curve Bottom,
		Curve Right,
		Curve Top,
		Curve Left,
		double MaxDeviationMM,
		int UCount,
		int VCount,
		int DegreeU,
		int DegreeV,
		int ProxyU,
		int ProxyV,
		double ProxyExtend,
		int MaxIterations,
		int VerifySamples,
		double NormalDotMin,
		int BoundarySamples,
		double BoundaryFairing,
		int HoleFillIterations,
		bool UseNetworkSurface,
		ref object UnifiedSurface,
		ref object ProxySurfaces,
		ref object MaxDevMM,
		ref object Passed,
		ref object Report,
		ref object DebugCurves,
		ref object DebugGrid)
  {
    UnifiedSurface = null;
    ProxySurfaces = null;
    MaxDevMM = null;
    Passed = false;
    Report = "";
    DebugCurves = null;
    DebugGrid = null;

    if (B == null || !B.IsValid)
    {
      Report = "Input Brep is null or invalid.";
      return;
    }

    if (B.Faces.Count < 1)
    {
      Report = "Input Brep has no faces.";
      return;
    }

    RhinoDoc doc = RhinoDocument;

    double modelTol = doc != null ? doc.ModelAbsoluteTolerance : 0.001;
    if (!RhinoMath.IsValidDouble(modelTol) || modelTol <= 0.0)
      modelTol = 0.001;

    double angleTol = doc != null ? doc.ModelAngleToleranceRadians : RhinoMath.ToRadians(1.0);
    if (!RhinoMath.IsValidDouble(angleTol) || angleTol <= 0.0)
      angleTol = RhinoMath.ToRadians(1.0);

    UnitSystem modelUnits = doc != null ? doc.ModelUnitSystem : UnitSystem.Millimeters;
    double mmToModel = RhinoMath.UnitScale(UnitSystem.Millimeters, modelUnits);
    if (!RhinoMath.IsValidDouble(mmToModel) || mmToModel <= 0.0)
      mmToModel = 1.0;

    if (MaxDeviationMM <= 0.0)
      MaxDeviationMM = 0.10;

    double targetTolModel = MaxDeviationMM * mmToModel;
    double workTol = Math.Max(modelTol, targetTolModel * 0.35);
    double boundaryTolModel = Math.Max(modelTol * 4.0, targetTolModel * 0.25);

    DegreeU = ClampInt(DegreeU <= 0 ? 3 : DegreeU, 1, 5);
    DegreeV = ClampInt(DegreeV <= 0 ? 3 : DegreeV, 1, 5);

    ProxyU = ClampInt(ProxyU <= 0 ? 70 : ProxyU, 12, 140);
    ProxyV = ClampInt(ProxyV <= 0 ? 70 : ProxyV, 12, 140);

    MaxIterations = ClampInt(MaxIterations <= 0 ? 3 : MaxIterations, 1, 5);
    VerifySamples = ClampInt(VerifySamples <= 0 ? 75 : VerifySamples, 10, 220);

    NormalDotMin = ClampDouble(NormalDotMin <= 0.0 ? 0.06 : NormalDotMin, 0.0, 0.95);
    BoundarySamples = ClampInt(BoundarySamples <= 0 ? 160 : BoundarySamples, 40, 500);
    double interiorFairing = ClampDouble(BoundaryFairing, 0.0, 0.85);
    HoleFillIterations = ClampInt(HoleFillIterations <= 0 ? 500 : HoleFillIterations, 20, 2500);

    BoundingBox originalBox = B.GetBoundingBox(true);
    if (!originalBox.IsValid || originalBox.Diagonal.Length <= RhinoMath.ZeroTolerance)
    {
      Report = "Input Brep has an invalid or near-zero bounding box.";
      return;
    }

    Point3d originalCenter = originalBox.Center;
    Transform toLocal = Transform.Translation(-originalCenter.X, -originalCenter.Y, -originalCenter.Z);
    Transform toWorld = Transform.Translation(originalCenter.X, originalCenter.Y, originalCenter.Z);

    Brep brep = B.DuplicateBrep();
    brep.Transform(toLocal);
    brep.Faces.ShrinkFaces();

    Curve bottomInput = DuplicateAndTransformCurve(Bottom, toLocal);
    Curve rightInput = DuplicateAndTransformCurve(Right, toLocal);
    Curve topInput = DuplicateAndTransformCurve(Top, toLocal);
    Curve leftInput = DuplicateAndTransformCurve(Left, toLocal);

    bool anyManualRailInput = Bottom != null || Right != null || Top != null || Left != null;
    bool allManualRailInputs = Bottom != null && Right != null && Top != null && Left != null;

    if (anyManualRailInput && !allManualRailInputs)
    {
      Report =
        "Partial manual rail input was supplied.\n\n" +
        "Supply all four manual rails, or supply none and let the script attempt auto-boundary extraction.\n\n" +
        "Required rails:\n" +
        "Bottom = lower base-skin edge\n" +
        "Right = right base-skin edge\n" +
        "Top = upper base-skin edge\n" +
        "Left = left base-skin edge";
      return;
    }

    bool manualRailsSupplied =
      allManualRailInputs &&
      bottomInput != null && bottomInput.IsValid &&
      rightInput != null && rightInput.IsValid &&
      topInput != null && topInput.IsValid &&
      leftInput != null && leftInput.IsValid;

    if (allManualRailInputs && !manualRailsSupplied)
    {
      Report =
        "All four manual rail inputs were supplied, but at least one curve is null or invalid.\n\n" +
        "Supply four valid open curves from the actual base-skin boundary.";
      return;
    }

    Plane panelPlane;
    List<Point3d> planePoints;

    if (!TryGetPanelPlane(brep, out panelPlane, out planePoints))
    {
      Report = "Could not calculate a stable panel plane.";
      return;
    }

    List<FitFace> allFitFaces;
    List<FitFace> selectedFitFaces;
    int rejectedByNormal;
    int failedFaceBuild;
    string faceFilterNote;

    BuildFitFaces(
      brep,
      panelPlane,
      NormalDotMin,
      Math.Min(ProxyU, 70),
      Math.Min(ProxyV, 70),
      out allFitFaces,
      out rejectedByNormal,
      out failedFaceBuild);

    SelectDominantFitFaces(allFitFaces, out selectedFitFaces, out faceFilterNote);

    if (selectedFitFaces == null || selectedFitFaces.Count < 1)
    {
      Report =
        "No usable base-skin faces were selected.\n" +
        "Lower NormalDotMin or isolate the base-skin faces before fitting.\n" +
        "Faces rejected by normal filter: " + rejectedByNormal + "\n" +
        "Faces failed sampling: " + failedFaceBuild;
      return;
    }

    List<Point3d> refinedPoints = CollectFitFacePoints(selectedFitFaces, 18);
    Plane refinedPlane;

    if (TryFitPlaneFromPoints(refinedPoints, out refinedPlane))
    {
      panelPlane = refinedPlane;

      List<FitFace> refinedAll;
      List<FitFace> refinedSelected;
      int refinedRejected;
      int refinedFailed;
      string refinedNote;

      BuildFitFaces(
        brep,
        panelPlane,
        NormalDotMin,
        Math.Min(ProxyU, 70),
        Math.Min(ProxyV, 70),
        out refinedAll,
        out refinedRejected,
        out refinedFailed);

      SelectDominantFitFaces(refinedAll, out refinedSelected, out refinedNote);

      if (refinedSelected != null && refinedSelected.Count > 0)
      {
        allFitFaces = refinedAll;
        selectedFitFaces = refinedSelected;
        rejectedByNormal = refinedRejected;
        failedFaceBuild = refinedFailed;
        faceFilterNote = refinedNote + " Projection plane was refined from selected base-skin faces.";
      }
    }

    List<SourceSample> sourceSamples = BuildSourceSamples(selectedFitFaces, panelPlane, ProxyU, ProxyV, NormalDotMin);

    if (sourceSamples.Count < 30)
      sourceSamples = BuildSourceSamples(selectedFitFaces, panelPlane, ProxyU, ProxyV, 0.0);

    if (sourceSamples.Count < 30)
    {
      Report =
        "Too few source samples were collected from the selected base-skin faces.\n" +
        "Try lowering NormalDotMin or supplying a cleaner Brep containing only the base skin.";
      return;
    }

    double sampleScale = EstimateSampleScale(sourceSamples);

    Curve[] rails = null;
    bool usedManualRails = false;
    bool usedSelectedOuterBoundary = false;
    string railReason = "";

    if (manualRailsSupplied)
    {
      usedManualRails = true;

      Curve[] manualInput = new Curve[4];
      manualInput[0] = bottomInput;
      manualInput[1] = rightInput;
      manualInput[2] = topInput;
      manualInput[3] = leftInput;

      Curve[] orderedManual;
      string manualReason;

      if (!AutoOrderAndOrientManualRails(manualInput, panelPlane, BoundarySamples, 0.0, out orderedManual, out manualReason))
      {
        Report =
          "Manual rails were supplied but could not be interpreted as one clean four-sided boundary.\n\n" +
          manualReason + "\n\n" +
          "Supply four open curves from the actual base-skin boundary:\n" +
          "Bottom = lower base-skin edge\n" +
          "Right = right base-skin edge\n" +
          "Top = upper base-skin edge\n" +
          "Left = left base-skin edge\n\n" +
          "Do not use return/flange/lip edges.";
        return;
      }

      rails = orderedManual;
      railReason =
        "Manual rails were used as hard network boundaries.\n" +
        "The boundary was also measured against the resulting surface after creation.";
    }
    else
    {
      Curve[] selectedBoundaryRails;
      string selectedBoundaryReason;
      List<Curve> autoBoundaryDebugLocal = new List<Curve>();

      if (!TryBuildOuterBoundaryRailsFromSelectedFaces(
        selectedFitFaces,
        panelPlane,
        BoundarySamples,
        modelTol * 8.0,
        out selectedBoundaryRails,
        out selectedBoundaryReason,
        autoBoundaryDebugLocal))
      {
        if (autoBoundaryDebugLocal != null && autoBoundaryDebugLocal.Count > 0)
          DebugCurves = DuplicateTransformCurves(autoBoundaryDebugLocal, toWorld);

        Report =
          "Auto boundary extraction failed, and envelope fallback is intentionally disabled.\n\n" +
          "Reason:\n" +
          selectedBoundaryReason + "\n\n" +
          "DebugCurves contains the recovered outer loop, attempted split rails, best failed rail ordering, and failed Coons grid if available.\n\n" +
          "Supply manual Bottom, Right, Top, and Left rails from the actual base-skin boundary.\n" +
          "Do not use flange/return/lip edges. Do not use the full Brep naked edge.\n\n" +
          "This stop is intentional: envelope fallback creates inconsistent edges.";
        return;
      }

      rails = selectedBoundaryRails;
      usedSelectedOuterBoundary = true;
      railReason =
        "Hard boundary rails were extracted from the selected base-skin faces.\n" +
        selectedBoundaryReason;
    }

    if (rails == null || rails.Length != 4)
    {
      Report = "Boundary rail creation failed.";
      return;
    }

    int startU = UCount;
    int startV = VCount;
    EstimateNetworkCounts(rails, DegreeU, DegreeV, ref startU, ref startV);

    BoundingBox bbox = brep.GetBoundingBox(true);
    double diag = bbox.IsValid ? bbox.Diagonal.Length : sampleScale;

    NurbsSurface bestSurface = null;
    Point3d[,] bestGrid = null;
    Curve[] bestRows = null;
    Curve[] bestColumns = null;

    double bestDeviation = double.MaxValue;
    double bestBoundaryDeviation = double.MaxValue;
    double bestRoughness = double.MaxValue;

    int bestU = startU;
    int bestV = startV;
    int bestMeasured = 0;
    int bestBoundaryMeasured = 0;
    int bestFaceHits = 0;
    int bestIdwHits = 0;
    int bestNetworkError = 0;
    string bestMethod = "";

    int currentU = startU;
    int currentV = startV;

    for (int iteration = 0; iteration < MaxIterations; iteration++)
    {
      Point3d[,] baseGrid = BuildCoonsGrid(rails, currentU, currentV);
      EnforceBoundaryGrid(baseGrid, rails);

      string foldReason;
      if (!ProjectedGridIsNotFolded(baseGrid, panelPlane, diag, out foldReason))
      {
        DebugGrid = DuplicateTransformCurves(MakeDebugGridCurves(baseGrid, 28), toWorld);
        Report =
          "The boundary UV frame folds over.\n" +
          foldReason + "\n\n" +
          "Use cleaner manual rails or isolate the base-skin region.";
        return;
      }

      Point3d[,] fittedGrid;
      bool[,] anchorMask;
      int faceHits;
      int idwHits;

      FitInteriorGridWithHardBoundary(
        baseGrid,
        rails,
        selectedFitFaces,
        sourceSamples,
        panelPlane,
        NormalDotMin,
        workTol,
        sampleScale,
        HoleFillIterations,
        out fittedGrid,
        out anchorMask,
        out faceHits,
        out idwHits);

      int fairPasses = ClampInt((int) Math.Round(10.0 + interiorFairing * 38.0), 0, 46);
      double anchorStrength = ClampDouble(2.3 - interiorFairing * 1.3, 0.65, 2.3);

      FairInteriorGridWithHardBoundary(
        baseGrid,
        fittedGrid,
        rails,
        anchorMask,
        panelPlane,
        fairPasses,
        interiorFairing,
        anchorStrength);

      EnforceBoundaryGrid(fittedGrid, rails);
      double roughness = ComputeGridRoughness(fittedGrid, panelPlane);

      Curve[] rows;
      Curve[] columns;
      MakeHardBoundaryNetworkCurves(fittedGrid, rails, DegreeU, DegreeV, out rows, out columns);

      if (rows == null || columns == null || rows.Length < 2 || columns.Length < 2)
      {
        Report = "Could not create row/column network curves.";
        return;
      }

      NurbsSurface candidate = null;
      int networkError = 0;
      string method = "";

      if (UseNetworkSurface)
      {
        candidate = CreateNetworkSurfaceFromCurves(
          rows,
          columns,
          Math.Max(modelTol, targetTolModel * 0.05),
          Math.Max(workTol, targetTolModel),
          angleTol,
          out networkError);

        method = "Hard boundary rails + tweened interior curves -> NetworkSurface";
      }

      if (candidate == null || !candidate.IsValid)
      {
        candidate = CreateThroughPointSurfaceFromGrid(fittedGrid, currentU, currentV, DegreeU, DegreeV);
        method = "Fallback ThroughPoints from hard-boundary sample grid; boundary was verified after creation";
      }

      if (candidate == null || !candidate.IsValid)
      {
        DebugGrid = DuplicateTransformCurves(MakeDebugGridCurves(fittedGrid, 28), toWorld);
        Report =
          "NetworkSurface and fallback ThroughPoints both failed.\n" +
          "Try lower UCount/VCount or degree 3.";
        return;
      }

      int measuredCount;
      double deviation = MeasureOriginalFacesToSurfaceDeviation(
        selectedFitFaces,
        candidate,
        VerifySamples,
        NormalDotMin,
        panelPlane.ZAxis,
        out measuredCount);

      if (measuredCount < 1)
      {
        currentU = ClampInt((int) Math.Ceiling(currentU * 1.16), currentU + 1, 46);
        currentV = ClampInt((int) Math.Ceiling(currentV * 1.16), currentV + 1, 46);
        continue;
      }

      int boundaryMeasured;
      double boundaryDeviation = MeasureSurfaceBoundaryToRailsDeviation(
        candidate,
        rails,
        Math.Max(BoundarySamples, VerifySamples),
        out boundaryMeasured);

      if (boundaryMeasured < 8)
        boundaryDeviation = double.MaxValue;

      bool currentPass =
        bestSurface != null &&
        bestDeviation <= targetTolModel &&
        bestBoundaryDeviation <= boundaryTolModel;

      bool candidatePass = deviation <= targetTolModel && boundaryDeviation <= boundaryTolModel;

      double targetSafe = Math.Max(targetTolModel, 1e-12);
      double boundarySafe = Math.Max(boundaryTolModel, 1e-12);

      double currentScore = bestSurface == null ? double.MaxValue : Math.Max(bestDeviation / targetSafe, bestBoundaryDeviation / boundarySafe);
      double candidateScore = Math.Max(deviation / targetSafe, boundaryDeviation / boundarySafe);

      bool take = false;

      if (bestSurface == null)
        take = true;
      else if (!currentPass && !candidatePass)
      {
        if (candidateScore < currentScore)
          take = true;
        else if (Math.Abs(candidateScore - currentScore) < 0.05 && roughness < bestRoughness)
          take = true;
      }
      else if (candidatePass && !currentPass)
        take = true;
      else if (candidatePass && currentPass)
      {
        if (roughness < bestRoughness && deviation <= bestDeviation * 1.35 && boundaryDeviation <= bestBoundaryDeviation * 1.35)
          take = true;
        else if (candidateScore < currentScore * 0.70)
          take = true;
      }

      if (take)
      {
        bestSurface = candidate;
        bestGrid = CloneGrid(fittedGrid);
        bestRows = DuplicateCurveArray(rows);
        bestColumns = DuplicateCurveArray(columns);

        bestDeviation = deviation;
        bestBoundaryDeviation = boundaryDeviation;
        bestRoughness = roughness;

        bestU = currentU;
        bestV = currentV;
        bestMeasured = measuredCount;
        bestBoundaryMeasured = boundaryMeasured;
        bestFaceHits = faceHits;
        bestIdwHits = idwHits;
        bestNetworkError = networkError;
        bestMethod = method;
      }

      if (bestSurface != null && bestDeviation <= targetTolModel && bestBoundaryDeviation <= boundaryTolModel)
        break;

      currentU = ClampInt((int) Math.Ceiling(currentU * 1.18), currentU + 1, 46);
      currentV = ClampInt((int) Math.Ceiling(currentV * 1.18), currentV + 1, 46);
    }

    if (bestSurface == null)
    {
      Report =
        "No surface could be verified against the selected base-skin faces.\n" +
        "Try lower NormalDotMin, lower UCount/VCount, or supply manual boundary rails.";
      return;
    }

    bestSurface.Transform(toWorld);

    List<Brep> selectedFacesWorld = new List<Brep>();
    for (int i = 0; i < selectedFitFaces.Count; i++)
    {
      if (selectedFitFaces[i] == null || selectedFitFaces[i].Face == null)
        continue;

      Brep fb = selectedFitFaces[i].Face.DuplicateFace(false);
      if (fb != null && fb.IsValid)
      {
        fb.Transform(toWorld);
        selectedFacesWorld.Add(fb);
      }
    }

    List<Curve> debugCurvesLocal = new List<Curve>();

    for (int i = 0; i < rails.Length; i++)
    {
      if (rails[i] != null && rails[i].IsValid)
        debugCurvesLocal.Add(rails[i].DuplicateCurve());
    }

    if (bestRows != null)
    {
      for (int i = 0; i < bestRows.Length; i++)
      {
        if (bestRows[i] != null && bestRows[i].IsValid)
          debugCurvesLocal.Add(bestRows[i].DuplicateCurve());
      }
    }

    if (bestColumns != null)
    {
      for (int i = 0; i < bestColumns.Length; i++)
      {
        if (bestColumns[i] != null && bestColumns[i].IsValid)
          debugCurvesLocal.Add(bestColumns[i].DuplicateCurve());
      }
    }

    DebugCurves = DuplicateTransformCurves(debugCurvesLocal, toWorld);

    if (bestGrid != null)
      DebugGrid = DuplicateTransformCurves(MakeDebugGridCurves(bestGrid, 28), toWorld);

    double selectedFaceDevMM = bestDeviation / mmToModel;
    double boundaryDevMM = bestBoundaryDeviation / mmToModel;
    double devMM = Math.Max(bestDeviation, bestBoundaryDeviation) / mmToModel;

    bool pass = bestDeviation <= targetTolModel && bestBoundaryDeviation <= boundaryTolModel;

    NurbsSurface bestSurfaceWorld = bestSurface.DuplicateSurface() as NurbsSurface;
    if (bestSurfaceWorld != null)
      bestSurfaceWorld.Transform(toWorld);
    else
      bestSurface.Transform(toWorld);

    UnifiedSurface = bestSurfaceWorld ?? bestSurface;
    ProxySurfaces = selectedFacesWorld;
    MaxDevMM = devMM;
    Passed = pass;

    string boundaryWarning = "";

    if (bestBoundaryDeviation > boundaryTolModel)
    {
      boundaryWarning =
        "WARNING: final surface boundary did not meet the rail-boundary tolerance.\n" +
        "The surface is output for inspection, but Passed is false.\n" +
        "Check DebugCurves first. If the rails do not overlay the base-skin edge, supply manual rails.\n\n";
    }

    Report =
      boundaryWarning +
      "Unified hard-boundary network surface process completed.\n" +
      "Passed target: " + pass + "\n" +
      "Target selected-face deviation: " + MaxDeviationMM.ToString("0.####") + " mm\n" +
      "Measured selected-face max deviation: " + selectedFaceDevMM.ToString("0.####") + " mm\n" +
      "Boundary tolerance used: " + (boundaryTolModel / mmToModel).ToString("0.####") + " mm\n" +
      "Measured boundary max deviation: " + boundaryDevMM.ToString("0.####") + " mm\n" +
      "Reported MaxDevMM is the worse of selected-face deviation and boundary deviation.\n" +
      "Measurement direction: selected trimmed base-skin faces -> final surface, plus final surface boundary <-> rails.\n\n" +
      "Method: " + bestMethod + "\n" +
      "NetworkSurface error code: " + bestNetworkError + "\n" +
      "Network curve counts: U " + bestU + ", V " + bestV + "\n" +
      "Degrees: U " + DegreeU + ", V " + DegreeV + "\n" +
      "Source sampling grid: " + ProxyU + " x " + ProxyV + "\n" +
      "NormalDotMin: " + NormalDotMin.ToString("0.###") + "\n" +
      "Interior fairing: " + interiorFairing.ToString("0.###") + "\n" +
      "HoleFillIterations used: " + HoleFillIterations + "\n" +
      "Grid roughness: " + bestRoughness.ToString("0.####") + "\n\n" +
      "Usable face candidates: " + allFitFaces.Count + "\n" +
      "Selected base-skin faces: " + selectedFitFaces.Count + "\n" +
      "Source samples: " + sourceSamples.Count + "\n" +
      "Face filter: " + faceFilterNote + "\n" +
      "Faces rejected by normal filter: " + rejectedByNormal + "\n" +
      "Faces failed sampling: " + failedFaceBuild + "\n" +
      "Interior samples from trimmed-face closest points: " + bestFaceHits + "\n" +
      "Interior samples from robust IDW: " + bestIdwHits + "\n" +
      "Measured selected face samples: " + bestMeasured + "\n" +
      "Measured boundary samples: " + bestBoundaryMeasured + "\n\n" +
      "Manual rails used: " + usedManualRails + "\n" +
      "Selected-face outer boundary used: " + usedSelectedOuterBoundary + "\n" +
      "Envelope fallback used: False\n" +
      "ProxyExtend input: ignored. Boundary extension/envelope fallback is intentionally disabled.\n\n" +
      "Rail note:\n" + railReason + "\n\n" +
      "The intended boundary is defined only by the four boundary rails.\n" +
      "Only the interior network is fitted and faired.\n" +
      "ProxySurfaces contains the selected trimmed source faces.\n" +
      "DebugCurves contains boundary rails plus final U/V network curves.\n" +
      "DebugGrid contains the final hard-boundary fitted grid.\n\n" +
      "Recommended start values:\n" +
      "UCount 24, VCount 12, DegreeU 3, DegreeV 3, ProxyU/ProxyV 70, BoundaryFairing 0 for edge testing, then 0.25-0.50, NormalDotMin 0.04-0.10.";
  }

  // --------------------------------------------------------------------------
  // Transform helpers
  // --------------------------------------------------------------------------

  private Curve DuplicateAndTransformCurve(Curve c, Transform xform)
  {
    if (c == null || !c.IsValid)
      return null;

    Curve d = c.DuplicateCurve();
    d.Transform(xform);
    return d;
  }

  private Curve[] DuplicateCurveArray(Curve[] curves)
  {
    if (curves == null)
      return null;

    Curve[] result = new Curve[curves.Length];
    for (int i = 0; i < curves.Length; i++)
    {
      if (curves[i] != null && curves[i].IsValid)
        result[i] = curves[i].DuplicateCurve();
    }

    return result;
  }

  private List<Curve> DuplicateTransformCurves(List<Curve> curves, Transform xform)
  {
    List<Curve> result = new List<Curve>();

    if (curves == null)
      return result;

    for (int i = 0; i < curves.Count; i++)
    {
      if (curves[i] == null || !curves[i].IsValid)
        continue;

      Curve c = curves[i].DuplicateCurve();
      c.Transform(xform);
      result.Add(c);
    }

    return result;
  }

  // --------------------------------------------------------------------------
  // Plane fitting
  // --------------------------------------------------------------------------

  private bool TryGetPanelPlane(Brep brep, out Plane plane, out List<Point3d> points)
  {
    plane = Plane.WorldXY;
    points = new List<Point3d>();

    for (int i = 0; i < brep.Vertices.Count; i++)
      points.Add(brep.Vertices[i].Location);

    foreach (BrepFace face in brep.Faces)
    {
      Interval du = face.Domain(0);
      Interval dv = face.Domain(1);
      int n = 7;

      for (int i = 0; i < n; i++)
      {
        double u = du.ParameterAt((double) i / (double) (n - 1));

        for (int j = 0; j < n; j++)
        {
          double v = dv.ParameterAt((double) j / (double) (n - 1));

          if (face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
            continue;

          Point3d p = face.PointAt(u, v);
          if (p.IsValid)
            points.Add(p);
        }
      }
    }

    return TryFitPlaneFromPoints(points, out plane);
  }

  private bool TryFitPlaneFromPoints(List<Point3d> points, out Plane plane)
  {
    plane = Plane.WorldXY;

    if (points == null || points.Count < 4)
      return false;

    Plane raw;
    PlaneFitResult fit = Plane.FitPlaneToPoints(points, out raw);

    if (fit == PlaneFitResult.Failure || !raw.IsValid)
      return false;

    plane = OrientPlaneByPca(raw, points);
    return plane.IsValid;
  }

  private Plane OrientPlaneByPca(Plane raw, List<Point3d> points)
  {
    double mx = 0.0;
    double my = 0.0;

    for (int i = 0; i < points.Count; i++)
    {
      double x;
      double y;
      raw.ClosestParameter(points[i], out x, out y);
      mx += x;
      my += y;
    }

    mx /= points.Count;
    my /= points.Count;

    double cxx = 0.0;
    double cxy = 0.0;
    double cyy = 0.0;

    for (int i = 0; i < points.Count; i++)
    {
      double x;
      double y;
      raw.ClosestParameter(points[i], out x, out y);
      x -= mx;
      y -= my;
      cxx += x * x;
      cxy += x * y;
      cyy += y * y;
    }

    double theta = 0.5 * Math.Atan2(2.0 * cxy, cxx - cyy);
    Vector3d xAxis = raw.XAxis * Math.Cos(theta) + raw.YAxis * Math.Sin(theta);

    if (!xAxis.Unitize())
      xAxis = raw.XAxis;

    Vector3d zAxis = raw.ZAxis;
    if (!zAxis.Unitize())
      zAxis = Vector3d.ZAxis;

    Vector3d yAxis = Vector3d.CrossProduct(zAxis, xAxis);
    if (!yAxis.Unitize())
      yAxis = raw.YAxis;

    return new Plane(raw.Origin, xAxis, yAxis);
  }

  // --------------------------------------------------------------------------
  // Fit face selection
  // --------------------------------------------------------------------------

  private void BuildFitFaces(
    Brep brep,
    Plane plane,
    double normalDotMin,
    int sampleU,
    int sampleV,
    out List<FitFace> fitFaces,
    out int rejectedByNormal,
    out int failedFaceBuild)
  {
    fitFaces = new List<FitFace>();
    rejectedByNormal = 0;
    failedFaceBuild = 0;

    sampleU = ClampInt(sampleU, 8, 90);
    sampleV = ClampInt(sampleV, 8, 90);

    Vector3d panelNormal = plane.ZAxis;
    if (!panelNormal.Unitize())
      panelNormal = Vector3d.ZAxis;

    for (int faceIndex = 0; faceIndex < brep.Faces.Count; faceIndex++)
    {
      BrepFace face = brep.Faces[faceIndex];
      double normalDot = EstimateFaceNormalDot(face, panelNormal, 6);

      if (normalDot < normalDotMin)
      {
        rejectedByNormal++;
        continue;
      }

      ProjectedBox box;
      double area = EstimateTrimmedFaceProjectedAreaAndBox(
        face,
        plane,
        Math.Min(sampleU, 55),
        Math.Min(sampleV, 55),
        out box);

      if (area <= RhinoMath.ZeroTolerance || box == null || !box.IsValid())
      {
        failedFaceBuild++;
        continue;
      }

      FitFace ff = new FitFace();
      ff.Face = face;
      ff.FaceIndex = faceIndex;
      ff.NormalDot = normalDot;
      ff.ProjectedArea = area;
      ff.Box = box;
      fitFaces.Add(ff);
    }
  }

  private void SelectDominantFitFaces(List<FitFace> candidates, out List<FitFace> selected, out string note)
  {
    selected = new List<FitFace>();
    note = "";

    if (candidates == null || candidates.Count == 0)
    {
      note = "No fit-face candidates were available.";
      return;
    }

    double largestArea = 0.0;
    double maxNormalDot = 0.0;

    for (int i = 0; i < candidates.Count; i++)
    {
      FitFace ff = candidates[i];
      if (ff == null)
        continue;

      if (ff.ProjectedArea > largestArea)
        largestArea = ff.ProjectedArea;
      if (ff.NormalDot > maxNormalDot)
        maxNormalDot = ff.NormalDot;
    }

    if (largestArea <= RhinoMath.ZeroTolerance)
    {
      selected.AddRange(candidates);
      note = "Projected areas were degenerate; all normal-compatible faces were selected.";
      return;
    }

    double minArea = largestArea * 0.018;
    double normalFloor = Math.Max(0.0, maxNormalDot - 0.34);

    for (int i = 0; i < candidates.Count; i++)
    {
      FitFace ff = candidates[i];
      if (ff == null)
        continue;
      if (ff.ProjectedArea < minArea)
        continue;
      if (ff.NormalDot < normalFloor)
        continue;
      selected.Add(ff);
    }

    if (selected.Count < 1)
    {
      double bestArea = -1.0;
      FitFace bestFace = null;

      for (int i = 0; i < candidates.Count; i++)
      {
        if (candidates[i] != null && candidates[i].ProjectedArea > bestArea)
        {
          bestArea = candidates[i].ProjectedArea;
          bestFace = candidates[i];
        }
      }

      if (bestFace != null)
        selected.Add(bestFace);
    }

    note =
      "Selected " + selected.Count + " of " + candidates.Count +
      " normal-compatible faces. Ignored likely small/detail/return faces: " + (candidates.Count - selected.Count) + ".";
  }

  private List<Point3d> CollectFitFacePoints(List<FitFace> faces, int samples)
  {
    List<Point3d> points = new List<Point3d>();

    if (faces == null)
      return points;

    samples = ClampInt(samples, 4, 80);

    for (int f = 0; f < faces.Count; f++)
    {
      if (faces[f] == null || faces[f].Face == null)
        continue;

      BrepFace face = faces[f].Face;
      Interval du = face.Domain(0);
      Interval dv = face.Domain(1);

      for (int i = 0; i < samples; i++)
      {
        double u = du.ParameterAt((double) i / (double) (samples - 1));

        for (int j = 0; j < samples; j++)
        {
          double v = dv.ParameterAt((double) j / (double) (samples - 1));

          if (face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
            continue;

          Point3d p = face.PointAt(u, v);
          if (p.IsValid)
            points.Add(p);
        }
      }
    }

    return points;
  }

  private double EstimateFaceNormalDot(BrepFace face, Vector3d panelNormal, int samples)
  {
    double sum = 0.0;
    int count = 0;

    samples = ClampInt(samples, 3, 20);
    Interval du = face.Domain(0);
    Interval dv = face.Domain(1);

    for (int i = 0; i < samples; i++)
    {
      double u = du.ParameterAt((double) i / (double) (samples - 1));

      for (int j = 0; j < samples; j++)
      {
        double v = dv.ParameterAt((double) j / (double) (samples - 1));

        if (face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
          continue;

        Vector3d n = face.NormalAt(u, v);
        if (!n.Unitize())
          continue;

        sum += Math.Abs(n * panelNormal);
        count++;
      }
    }

    if (count == 0)
      return 0.0;

    return sum / (double) count;
  }

  private double EstimateTrimmedFaceProjectedAreaAndBox(
    BrepFace face,
    Plane plane,
    int uSamples,
    int vSamples,
    out ProjectedBox box)
  {
    box = new ProjectedBox();
    uSamples = ClampInt(uSamples, 3, 90);
    vSamples = ClampInt(vSamples, 3, 90);

    Interval du = face.Domain(0);
    Interval dv = face.Domain(1);

    Point3d[,] pts = new Point3d[uSamples, vSamples];
    bool[,] valid = new bool[uSamples, vSamples];

    for (int v = 0; v < vSamples; v++)
    {
      double vv = dv.ParameterAt((double) v / (double) (vSamples - 1));

      for (int u = 0; u < uSamples; u++)
      {
        double uu = du.ParameterAt((double) u / (double) (uSamples - 1));

        if (face.IsPointOnFace(uu, vv) == PointFaceRelation.Exterior)
        {
          valid[u, v] = false;
          continue;
        }

        Point3d p = face.PointAt(uu, vv);
        if (!p.IsValid)
        {
          valid[u, v] = false;
          continue;
        }

        pts[u, v] = p;
        valid[u, v] = true;

        double x;
        double y;
        plane.ClosestParameter(p, out x, out y);
        box.Include(x, y);
      }
    }

    double area = 0.0;

    for (int v = 0; v < vSamples - 1; v++)
    {
      for (int u = 0; u < uSamples - 1; u++)
      {
        if (!valid[u, v] || !valid[u + 1, v] || !valid[u + 1, v + 1] || !valid[u, v + 1])
          continue;

        area += Math.Abs(ProjectedQuadArea(
          pts[u, v],
          pts[u + 1, v],
          pts[u + 1, v + 1],
          pts[u, v + 1],
          plane));
      }
    }

    if (!RhinoMath.IsValidDouble(area))
      return 0.0;

    return area;
  }

  private List<SourceSample> BuildSourceSamples(
    List<FitFace> faces,
    Plane plane,
    int uSamples,
    int vSamples,
    double normalDotMin)
  {
    List<SourceSample> samples = new List<SourceSample>();

    if (faces == null)
      return samples;

    uSamples = ClampInt(uSamples, 8, 140);
    vSamples = ClampInt(vSamples, 8, 140);

    Vector3d panelNormal = plane.ZAxis;
    if (!panelNormal.Unitize())
      panelNormal = Vector3d.ZAxis;

    for (int f = 0; f < faces.Count; f++)
    {
      if (faces[f] == null || faces[f].Face == null)
        continue;

      BrepFace face = faces[f].Face;
      Interval du = face.Domain(0);
      Interval dv = face.Domain(1);

      for (int i = 0; i < uSamples; i++)
      {
        double u = du.ParameterAt((double) i / (double) (uSamples - 1));

        for (int j = 0; j < vSamples; j++)
        {
          double v = dv.ParameterAt((double) j / (double) (vSamples - 1));

          if (face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
            continue;

          Vector3d n = face.NormalAt(u, v);
          if (!n.Unitize())
            continue;

          double dot = Math.Abs(n * panelNormal);
          if (dot < normalDotMin)
            continue;

          Point3d p = face.PointAt(u, v);
          if (!p.IsValid)
            continue;

          double x;
          double y;
          plane.ClosestParameter(p, out x, out y);

          SourceSample s = new SourceSample();
          s.Point = p;
          s.X = x;
          s.Y = y;
          s.H = PlaneHeight(p, plane);
          s.FaceIndex = faces[f].FaceIndex;
          samples.Add(s);
        }
      }
    }

    return samples;
  }

  // --------------------------------------------------------------------------
  // Boundary extraction
  // --------------------------------------------------------------------------

  private bool TryBuildOuterBoundaryRailsFromSelectedFaces(
    List<FitFace> selectedFaces,
    Plane plane,
    int boundarySamples,
    double tol,
    out Curve[] rails,
    out string reason,
    List<Curve> debugCurves)
  {
    rails = null;
    reason = "";

    if (debugCurves != null)
      debugCurves.Clear();

    if (selectedFaces == null || selectedFaces.Count < 1)
    {
      reason = "No selected faces were available for outer-boundary extraction.";
      return false;
    }

    List<Brep> faceBreps = new List<Brep>();

    for (int i = 0; i < selectedFaces.Count; i++)
    {
      if (selectedFaces[i] == null || selectedFaces[i].Face == null)
        continue;

      Brep fb = selectedFaces[i].Face.DuplicateFace(false);
      if (fb != null && fb.IsValid)
        faceBreps.Add(fb);
    }

    if (faceBreps.Count < 1)
    {
      reason = "Could not duplicate selected trimmed faces.";
      return false;
    }

    Brep[] joined = null;
    try { joined = Brep.JoinBreps(faceBreps.ToArray(), tol); }
    catch { joined = null; }

    List<Curve> candidateEdges = new List<Curve>();

    if (joined != null && joined.Length > 0)
    {
      for (int i = 0; i < joined.Length; i++)
      {
        if (joined[i] == null || !joined[i].IsValid)
          continue;

        Curve[] naked = joined[i].DuplicateNakedEdgeCurves(true, false);
        if (naked == null || naked.Length == 0)
          naked = joined[i].DuplicateNakedEdgeCurves(true, true);
        if (naked == null)
          continue;

        for (int k = 0; k < naked.Length; k++)
        {
          if (naked[k] != null && naked[k].IsValid)
            candidateEdges.Add(naked[k]);
        }
      }
    }

    if (candidateEdges.Count < 1)
    {
      for (int i = 0; i < faceBreps.Count; i++)
      {
        Curve[] naked = faceBreps[i].DuplicateNakedEdgeCurves(true, false);
        if (naked == null || naked.Length == 0)
          naked = faceBreps[i].DuplicateNakedEdgeCurves(true, true);
        if (naked == null)
          continue;

        for (int k = 0; k < naked.Length; k++)
        {
          if (naked[k] != null && naked[k].IsValid)
            candidateEdges.Add(naked[k]);
        }
      }
    }

    if (candidateEdges.Count < 1)
    {
      reason = "No naked edge curves were found on the selected faces.";
      return false;
    }

    Curve[] joinedCurves = null;
    try { joinedCurves = Curve.JoinCurves(candidateEdges, tol); }
    catch { joinedCurves = null; }

    if (joinedCurves == null || joinedCurves.Length < 1)
    {
      reason = "Selected face boundary curves could not be joined.";
      return false;
    }

    Curve bestLoop = null;
    double bestScore = -1.0;

    for (int i = 0; i < joinedCurves.Length; i++)
    {
      Curve c = joinedCurves[i];
      if (c == null || !c.IsValid)
        continue;

      Curve candidate = c;
      if (!candidate.IsClosed)
        candidate = TryCloseSmallGap(candidate, tol * 10.0);

      if (candidate == null || !candidate.IsValid || !candidate.IsClosed)
        continue;

      double area = Math.Abs(ClosedCurveProjectedArea(candidate, plane, 1000));
      double len = SafeCurveLength(candidate);
      double score = area > RhinoMath.ZeroTolerance ? area : len;

      if (score > bestScore)
      {
        bestScore = score;
        bestLoop = candidate.DuplicateCurve();

        if (debugCurves != null)
        {
          debugCurves.Clear();
          debugCurves.Add(bestLoop.DuplicateCurve());
        }
      }
    }

    if (bestLoop == null)
    {
      if (debugCurves != null)
      {
        debugCurves.Clear();
        for (int i = 0; i < joinedCurves.Length; i++)
        {
          if (joinedCurves[i] != null && joinedCurves[i].IsValid)
            debugCurves.Add(joinedCurves[i].DuplicateCurve());
        }
      }

      reason =
        "No closed outer boundary loop could be recovered from selected faces. " +
        "This usually means the selected faces are not joined into one clean base-skin patch, or the selected boundary is open.";
      return false;
    }

    Curve[] splitRails;
    string splitReason;

    if (!TrySplitClosedBoundaryIntoFourRails(
      bestLoop,
      plane,
      boundarySamples,
      tol,
      out splitRails,
      out splitReason,
      debugCurves))
    {
      reason =
        "Selected-face outer loop was found, but it could not be split into four rails.\n" +
        splitReason;
      return false;
    }

    rails = splitRails;
    reason = "Outer boundary was extracted from selected trimmed base-skin faces and split into four hard rails.";
    return true;
  }

  private bool TrySplitClosedBoundaryIntoFourRails(
    Curve boundary,
    Plane plane,
    int boundarySamples,
    double tol,
    out Curve[] rails,
    out string reason,
    List<Curve> debugCurves)
  {
    rails = null;
    reason = "";

    if (boundary == null || !boundary.IsValid || !boundary.IsClosed)
    {
      reason = "Boundary curve is not a valid closed curve.";
      return false;
    }

    if (debugCurves != null && debugCurves.Count == 0)
      debugCurves.Add(boundary.DuplicateCurve());

    double[] tCorners;
    if (!FindFourCornerParameters(boundary, plane, out tCorners, out reason))
      return false;

    List<CornerParam> ordered = new List<CornerParam>();
    for (int i = 0; i < 4; i++)
    {
      CornerParam cp = new CornerParam();
      cp.T = tCorners[i];
      cp.N = NormalizedCurveLengthAtParameter(boundary, tCorners[i]);
      ordered.Add(cp);
    }

    ordered.Sort(delegate(CornerParam a, CornerParam b)
    {
      if (a.N < b.N) return -1;
      if (a.N > b.N) return 1;
      return 0;
    });

    Curve[] loopSegments = new Curve[4];

    for (int i = 0; i < 4; i++)
    {
      double t0 = ordered[i].T;
      double t1 = ordered[(i + 1) % 4].T;
      Curve seg = ExtractForwardSubCurve(boundary, t0, t1, tol);

      if (seg == null || !seg.IsValid || SafeCurveLength(seg) <= RhinoMath.ZeroTolerance)
      {
        reason = "Failed to extract consecutive boundary segment " + i + " from the closed outer loop.";
        return false;
      }

      loopSegments[i] = seg;
    }

    if (debugCurves != null)
    {
      for (int i = 0; i < loopSegments.Length; i++)
      {
        if (loopSegments[i] != null && loopSegments[i].IsValid)
          debugCurves.Add(loopSegments[i].DuplicateCurve());
      }
    }

    Curve[] orderedRails;
    string orderReason;

    if (!AutoOrderAndOrientManualRails(loopSegments, plane, boundarySamples, 0.0, out orderedRails, out orderReason))
    {
      if (debugCurves != null && orderedRails != null && orderedRails.Length == 4)
      {
        for (int i = 0; i < orderedRails.Length; i++)
        {
          if (orderedRails[i] != null && orderedRails[i].IsValid)
            debugCurves.Add(orderedRails[i].DuplicateCurve());
        }

        Point3d[,] failedGrid18 = BuildCoonsGrid(orderedRails, 18, 18);
        List<Curve> failedGridCurves = MakeDebugGridCurves(failedGrid18, 18);

        for (int i = 0; i < failedGridCurves.Count; i++)
        {
          if (failedGridCurves[i] != null && failedGridCurves[i].IsValid)
            debugCurves.Add(failedGridCurves[i].DuplicateCurve());
        }

        int failedFlips;
        int failedTiny;
        GetProjectedGridFoldStats(failedGrid18, plane, out failedFlips, out failedTiny);

        reason =
          "Outer loop was split into four consecutive segments, but those segments could not be ordered as a non-folding UV frame.\n" +
          orderReason + "\n\n" +
          "DebugCurves now contains:\n" +
          "- recovered closed outer loop\n" +
          "- four raw split segments\n" +
          "- best failed ordered rail set\n" +
          "- failed 18 x 18 Coons grid curves\n\n" +
          "Best failed Coons frame stats: flipped cells " + failedFlips + ", near-zero cells " + failedTiny + ".\n\n" +
          "This usually means the selected outer loop is not truly four-sided, or the selected faces still include return/flange geometry.";
      }
      else
      {
        reason =
          "Outer loop was split into four consecutive segments, but those segments could not be ordered as a non-folding UV frame.\n" +
          orderReason + "\n\n" +
          "Only the recovered loop and raw split segments are available for debug because no best rail ordering was produced.\n\n" +
          "This usually means the selected outer loop is not truly four-sided, or the selected faces still include return/flange geometry.";
      }

      return false;
    }

    rails = orderedRails;

    if (debugCurves != null)
    {
      for (int i = 0; i < orderedRails.Length; i++)
      {
        if (orderedRails[i] != null && orderedRails[i].IsValid)
          debugCurves.Add(orderedRails[i].DuplicateCurve());
      }
    }

    int flips18;
    int tiny18;
    Point3d[,] testGrid18 = BuildCoonsGrid(rails, 18, 18);
    GetProjectedGridFoldStats(testGrid18, plane, out flips18, out tiny18);

    if (flips18 > 0)
    {
      if (debugCurves != null)
      {
        List<Curve> gridCurves = MakeDebugGridCurves(testGrid18, 18);
        for (int i = 0; i < gridCurves.Count; i++)
        {
          if (gridCurves[i] != null && gridCurves[i].IsValid)
            debugCurves.Add(gridCurves[i].DuplicateCurve());
        }
      }

      reason =
        "The ordered boundary rails still create a folded UV frame at 18 x 18. " +
        "Flipped cells: " + flips18 + ". Near-zero cells: " + tiny18 + ".";
      return false;
    }

    int flips36;
    int tiny36;
    Point3d[,] testGrid36 = BuildCoonsGrid(rails, 36, 36);
    GetProjectedGridFoldStats(testGrid36, plane, out flips36, out tiny36);

    if (flips36 > 0)
    {
      if (debugCurves != null)
      {
        List<Curve> gridCurves = MakeDebugGridCurves(testGrid36, 18);
        for (int i = 0; i < gridCurves.Count; i++)
        {
          if (gridCurves[i] != null && gridCurves[i].IsValid)
            debugCurves.Add(gridCurves[i].DuplicateCurve());
        }
      }

      reason =
        "The ordered boundary rails pass a coarse fold check but fail a denser 36 x 36 check. " +
        "Flipped cells: " + flips36 + ". Near-zero cells: " + tiny36 + ". " +
        "Manual rails are recommended.";
      return false;
    }

    reason =
      "Outer boundary was extracted from selected trimmed base-skin faces, split by cyclic loop order, " +
      "and auto-ordered into Bottom/Right/Top/Left hard rails.";
    return true;
  }

  private bool FindFourCornerParameters(Curve boundary, Plane plane, out double[] tCorners, out string reason)
  {
    tCorners = null;
    reason = "";

    int n = 2600;
    List<double> ts = new List<double>();
    List<double> xs = new List<double>();
    List<double> ys = new List<double>();

    double minX = double.MaxValue;
    double maxX = -double.MaxValue;
    double minY = double.MaxValue;
    double maxY = -double.MaxValue;

    for (int i = 0; i < n; i++)
    {
      double f = (double) i / (double) n;
      Point3d p = PointAtNormalizedLengthSafe(boundary, f);

      double t;
      if (!boundary.ClosestPoint(p, out t))
        t = boundary.Domain.ParameterAt(f);

      double x;
      double y;
      plane.ClosestParameter(p, out x, out y);

      ts.Add(t);
      xs.Add(x);
      ys.Add(y);

      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }

    double[,] targets = new double[4, 2];
    targets[0, 0] = minX; targets[0, 1] = minY;
    targets[1, 0] = maxX; targets[1, 1] = minY;
    targets[2, 0] = maxX; targets[2, 1] = maxY;
    targets[3, 0] = minX; targets[3, 1] = maxY;

    double[] byBox = new double[4];

    for (int k = 0; k < 4; k++)
    {
      double best = double.MaxValue;
      int bestIndex = 0;

      for (int i = 0; i < n; i++)
      {
        double dx = xs[i] - targets[k, 0];
        double dy = ys[i] - targets[k, 1];
        double d2 = dx * dx + dy * dy;

        if (d2 < best)
        {
          best = d2;
          bestIndex = i;
        }
      }

      byBox[k] = ts[bestIndex];
    }

    if (CornerParamsAreDistinct(boundary, byBox, 0.030))
    {
      tCorners = byBox;
      return true;
    }

    reason = "Automatic four-corner detection failed. Supply manual Bottom/Right/Top/Left rails from the original base-skin boundary.";
    return false;
  }

  private bool CornerParamsAreDistinct(Curve curve, double[] t, double minNormalizedGap)
  {
    if (t == null || t.Length != 4)
      return false;

    double[] a = new double[4];
    for (int i = 0; i < 4; i++)
      a[i] = NormalizedCurveLengthAtParameter(curve, t[i]);

    Array.Sort(a);

    for (int i = 0; i < 4; i++)
    {
      double x = a[i];
      double y = i == 3 ? a[0] + 1.0 : a[i + 1];

      if (y - x < minNormalizedGap)
        return false;
    }

    return true;
  }

  private Curve ExtractForwardSubCurve(Curve closed, double t0, double t1, double tol)
  {
    if (closed == null)
      return null;

    Interval d = closed.Domain;

    if (t1 > t0)
      return closed.Trim(t0, t1);

    Curve a = closed.Trim(t0, d.T1);
    Curve b = closed.Trim(d.T0, t1);

    List<Curve> parts = new List<Curve>();
    if (a != null) parts.Add(a);
    if (b != null) parts.Add(b);

    if (parts.Count == 0)
      return null;
    if (parts.Count == 1)
      return parts[0];

    Curve[] joined = Curve.JoinCurves(parts, tol);
    if (joined != null && joined.Length > 0)
      return joined[0];

    return null;
  }

  private Curve TryCloseSmallGap(Curve openCurve, double tol)
  {
    if (openCurve == null)
      return null;

    if (openCurve.IsClosed)
      return openCurve;

    double gap = openCurve.PointAtStart.DistanceTo(openCurve.PointAtEnd);
    if (gap > tol)
      return null;

    List<Curve> parts = new List<Curve>();
    parts.Add(openCurve.DuplicateCurve());
    parts.Add(new LineCurve(openCurve.PointAtEnd, openCurve.PointAtStart));

    Curve[] joined = Curve.JoinCurves(parts, tol);
    if (joined != null && joined.Length > 0 && joined[0] != null && joined[0].IsClosed)
      return joined[0];

    return null;
  }

  private double ClosedCurveProjectedArea(Curve c, Plane plane, int samples)
  {
    if (c == null || !c.IsValid)
      return 0.0;

    samples = ClampInt(samples, 20, 3000);
    double area = 0.0;

    double prevX = 0.0;
    double prevY = 0.0;
    double firstX = 0.0;
    double firstY = 0.0;

    for (int i = 0; i < samples; i++)
    {
      double f = (double) i / (double) samples;
      Point3d p = PointAtNormalizedLengthSafe(c, f);

      double x;
      double y;
      plane.ClosestParameter(p, out x, out y);

      if (i == 0)
      {
        firstX = x;
        firstY = y;
      }
      else
      {
        area += prevX * y - prevY * x;
      }

      prevX = x;
      prevY = y;
    }

    area += prevX * firstY - prevY * firstX;
    return 0.5 * area;
  }

  // --------------------------------------------------------------------------
  // Manual rail ordering
  // --------------------------------------------------------------------------

  private bool AutoOrderAndOrientManualRails(
    Curve[] inputRails,
    Plane plane,
    int boundarySamples,
    double boundaryFairing,
    out Curve[] orderedRails,
    out string reason)
  {
    orderedRails = null;
    reason = "";

    if (inputRails == null || inputRails.Length != 4)
    {
      reason = "Expected exactly four manual rails.";
      return false;
    }

    Curve[] clean = new Curve[4];

    for (int i = 0; i < 4; i++)
    {
      if (inputRails[i] == null || !inputRails[i].IsValid)
      {
        reason = "Manual rail " + i + " is null or invalid.";
        return false;
      }

      clean[i] = CreateFairedInterpolatedSide(inputRails[i], boundarySamples, boundaryFairing);

      if (clean[i] == null || !clean[i].IsValid || SafeCurveLength(clean[i]) <= RhinoMath.ZeroTolerance)
      {
        reason = "Manual rail " + i + " failed rebuild.";
        return false;
      }

      if (clean[i].IsClosed)
      {
        reason =
          "Manual rail " + i + " is closed. " +
          "Supply one open side rail, not a closed loop or full Brep naked edge.";
        return false;
      }
    }

    int[,] perms = new int[,]
    {
      {0,1,2,3}, {0,1,3,2}, {0,2,1,3}, {0,2,3,1}, {0,3,1,2}, {0,3,2,1},
      {1,0,2,3}, {1,0,3,2}, {1,2,0,3}, {1,2,3,0}, {1,3,0,2}, {1,3,2,0},
      {2,0,1,3}, {2,0,3,1}, {2,1,0,3}, {2,1,3,0}, {2,3,0,1}, {2,3,1,0},
      {3,0,1,2}, {3,0,2,1}, {3,1,0,2}, {3,1,2,0}, {3,2,0,1}, {3,2,1,0}
    };

    double bestScore = double.MaxValue;
    int bestFlips = int.MaxValue;
    int bestTiny = int.MaxValue;
    double bestClosure = double.MaxValue;
    Curve[] best = null;

    for (int p = 0; p < 24; p++)
    {
      for (int bits = 0; bits < 16; bits++)
      {
        Curve[] candidate = new Curve[4];

        for (int slot = 0; slot < 4; slot++)
        {
          int sourceIndex = perms[p, slot];
          candidate[slot] = clean[sourceIndex].DuplicateCurve();

          if ((bits & (1 << slot)) != 0)
            candidate[slot].Reverse();
        }

        double avgLen = AverageRailLength(candidate);
        if (avgLen <= RhinoMath.ZeroTolerance)
          continue;

        double closure = ManualRailClosureScore(candidate) / avgLen;

        Point3d[,] testGrid = BuildCoonsGrid(candidate, 18, 18);
        int flips;
        int tiny;
        GetProjectedGridFoldStats(testGrid, plane, out flips, out tiny);

        double score = flips * 1000000.0 + tiny * 1000.0 + closure;

        if (score < bestScore)
        {
          bestScore = score;
          bestFlips = flips;
          bestTiny = tiny;
          bestClosure = closure;
          best = new Curve[4];

          for (int i = 0; i < 4; i++)
            best[i] = candidate[i].DuplicateCurve();
        }
      }
    }

    if (best == null)
    {
      reason = "Could not find a usable orientation/permutation for the four manual rails.";
      return false;
    }

    orderedRails = best;

    if (bestFlips > 0)
    {
      reason =
        "Best manual rail ordering still creates a folded UV frame. " +
        "Flipped cells: " + bestFlips + ". Near-zero cells: " + bestTiny + ".";
      return false;
    }

    if (bestClosure > 0.08)
    {
      reason =
        "Manual rails were ordered, but their endpoints do not close cleanly enough for hard-boundary surfacing. " +
        "Normalized closure error: " + bestClosure.ToString("0.###") + ".";
      return false;
    }

    return true;
  }

  private double ManualRailClosureScore(Curve[] rails)
  {
    if (rails == null || rails.Length != 4)
      return double.MaxValue;
    if (rails[0] == null || rails[1] == null || rails[2] == null || rails[3] == null)
      return double.MaxValue;

    double score = 0.0;
    score += rails[0].PointAtStart.DistanceTo(rails[3].PointAtStart);
    score += rails[0].PointAtEnd.DistanceTo(rails[1].PointAtStart);
    score += rails[1].PointAtEnd.DistanceTo(rails[2].PointAtEnd);
    score += rails[3].PointAtEnd.DistanceTo(rails[2].PointAtStart);
    return score;
  }

  private double AverageRailLength(Curve[] rails)
  {
    if (rails == null || rails.Length == 0)
      return 0.0;

    double sum = 0.0;
    int count = 0;

    for (int i = 0; i < rails.Length; i++)
    {
      double len = SafeCurveLength(rails[i]);
      if (len > RhinoMath.ZeroTolerance)
      {
        sum += len;
        count++;
      }
    }

    if (count == 0)
      return 0.0;

    return sum / count;
  }

  private Curve CreateFairedInterpolatedSide(Curve raw, int sampleCount, double fairing)
  {
    if (raw == null || !raw.IsValid)
      return null;

    if (fairing <= 0.0)
      return raw.DuplicateCurve();

    sampleCount = ClampInt(sampleCount, 8, 500);
    List<Point3d> pts = new List<Point3d>();

    for (int i = 0; i < sampleCount; i++)
    {
      double f = (double) i / (double) (sampleCount - 1);
      pts.Add(PointAtNormalizedLengthSafe(raw, f));
    }

    CullClosePointsInPlace(pts, 1e-9);

    if (pts.Count < 4)
      return raw.DuplicateCurve();

    SmoothOpenPointListInPlace(pts, fairing);
    return CreateInterpolatedCurveSafe(pts, 3);
  }

  // --------------------------------------------------------------------------
  // Grid construction and fitting
  // --------------------------------------------------------------------------

  private Point3d[,] BuildCoonsGrid(Curve[] rails, int uCount, int vCount)
  {
    Point3d[] bottom = SampleCurve(rails[0], uCount);
    Point3d[] right = SampleCurve(rails[1], vCount);
    Point3d[] top = SampleCurve(rails[2], uCount);
    Point3d[] left = SampleCurve(rails[3], vCount);

    Point3d p00 = Average(bottom[0], left[0]);
    Point3d p10 = Average(bottom[uCount - 1], right[0]);
    Point3d p11 = Average(top[uCount - 1], right[vCount - 1]);
    Point3d p01 = Average(top[0], left[vCount - 1]);

    bottom[0] = p00;
    left[0] = p00;
    bottom[uCount - 1] = p10;
    right[0] = p10;
    top[uCount - 1] = p11;
    right[vCount - 1] = p11;
    top[0] = p01;
    left[vCount - 1] = p01;

    Point3d[,] grid = new Point3d[uCount, vCount];

    for (int v = 0; v < vCount; v++)
    {
      double tv = (double) v / (double) (vCount - 1);

      for (int u = 0; u < uCount; u++)
      {
        double tu = (double) u / (double) (uCount - 1);

        Point3d cb = bottom[u];
        Point3d ct = top[u];
        Point3d cl = left[v];
        Point3d cr = right[v];

        Point3d ruledA = Blend(cb, ct, tv);
        Point3d ruledB = Blend(cl, cr, tu);

        Point3d bilinear = Blend(Blend(p00, p10, tu), Blend(p01, p11, tu), tv);

        grid[u, v] = new Point3d(
          ruledA.X + ruledB.X - bilinear.X,
          ruledA.Y + ruledB.Y - bilinear.Y,
          ruledA.Z + ruledB.Z - bilinear.Z);
      }
    }

    return grid;
  }

  private void EnforceBoundaryGrid(Point3d[,] grid, Curve[] rails)
  {
    if (grid == null || rails == null || rails.Length != 4)
      return;

    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    Point3d[] bottom = SampleCurve(rails[0], uCount);
    Point3d[] right = SampleCurve(rails[1], vCount);
    Point3d[] top = SampleCurve(rails[2], uCount);
    Point3d[] left = SampleCurve(rails[3], vCount);

    for (int u = 0; u < uCount; u++)
    {
      grid[u, 0] = bottom[u];
      grid[u, vCount - 1] = top[u];
    }

    for (int v = 0; v < vCount; v++)
    {
      grid[0, v] = left[v];
      grid[uCount - 1, v] = right[v];
    }

    grid[0, 0] = Average(bottom[0], left[0]);
    grid[uCount - 1, 0] = Average(bottom[uCount - 1], right[0]);
    grid[uCount - 1, vCount - 1] = Average(top[uCount - 1], right[vCount - 1]);
    grid[0, vCount - 1] = Average(top[0], left[vCount - 1]);
  }

  private void FitInteriorGridWithHardBoundary(
    Point3d[,] baseGrid,
    Curve[] rails,
    List<FitFace> fitFaces,
    List<SourceSample> sourceSamples,
    Plane plane,
    double normalDotMin,
    double tol,
    double sampleScale,
    int holeFillIterations,
    out Point3d[,] fittedGrid,
    out bool[,] anchorMask,
    out int faceHits,
    out int idwHits)
  {
    int uCount = baseGrid.GetLength(0);
    int vCount = baseGrid.GetLength(1);

    fittedGrid = CloneGrid(baseGrid);
    anchorMask = new bool[uCount, vCount];
    faceHits = 0;
    idwHits = 0;

    double cell = EstimateProjectedCellSize(baseGrid, plane);
    double faceSnap = Math.Max(cell * 2.0, tol * 18.0);
    double boxPad = Math.Max(cell * 2.4, tol * 20.0);
    double idwRadius = Math.Max(cell * 6.0, sampleScale * 0.070);

    double edgeBand = 0.13;
    double edgeDeadZone = 0.018;

    for (int v = 0; v < vCount; v++)
    {
      double tv = (double) v / (double) (vCount - 1);

      for (int u = 0; u < uCount; u++)
      {
        double tu = (double) u / (double) (uCount - 1);
        bool edge = u == 0 || v == 0 || u == uCount - 1 || v == vCount - 1;

        if (edge)
        {
          anchorMask[u, v] = true;
          continue;
        }

        double edgeDistance = Math.Min(Math.Min(tu, 1.0 - tu), Math.Min(tv, 1.0 - tv));
        Point3d seed = baseGrid[u, v];
        double baseH = PlaneHeight(seed, plane);

        double x;
        double y;
        plane.ClosestParameter(seed, out x, out y);

        bool nearEdge = edgeDistance < edgeBand;
        double targetHeight = baseH;
        bool gotTarget = false;

        if (!nearEdge)
        {
          Point3d hit;
          if (TrySampleBestTrimmedFaceAt(seed, fitFaces, plane, normalDotMin, faceSnap, boxPad, out hit))
          {
            targetHeight = PlaneHeight(hit, plane);
            gotTarget = true;
            faceHits++;
          }
        }

        if (!gotTarget)
        {
          double idwH;
          if (TryEstimateHeightRobust(x, y, sourceSamples, idwRadius, sampleScale, out idwH))
          {
            targetHeight = idwH;
            gotTarget = true;
            idwHits++;
          }
        }

        if (gotTarget)
        {
          if (nearEdge)
          {
            double t = SmoothStep(ClampDouble((edgeDistance - edgeDeadZone) / Math.Max(1e-9, edgeBand - edgeDeadZone), 0.0, 1.0));
            targetHeight = baseH + (targetHeight - baseH) * t;
          }

          fittedGrid[u, v] = SetPointHeight(seed, targetHeight, plane);
          anchorMask[u, v] = true;
        }
        else
        {
          fittedGrid[u, v] = seed;
          anchorMask[u, v] = false;
        }
      }
    }

    InpaintMissingInteriorHeights(baseGrid, fittedGrid, anchorMask, plane, holeFillIterations);
    EnforceBoundaryGrid(fittedGrid, rails);
  }

  private bool TrySampleBestTrimmedFaceAt(
    Point3d seed,
    List<FitFace> fitFaces,
    Plane plane,
    double normalDotMin,
    double maxPlanarSnap,
    double boxPad,
    out Point3d hit)
  {
    hit = Point3d.Unset;

    if (fitFaces == null || fitFaces.Count == 0)
      return false;

    double sx;
    double sy;
    plane.ClosestParameter(seed, out sx, out sy);

    bool found = false;
    double bestMetric = double.MaxValue;

    for (int i = 0; i < fitFaces.Count; i++)
    {
      FitFace ff = fitFaces[i];
      if (ff == null || ff.Face == null || ff.Box == null)
        continue;
      if (!ff.Box.Contains(sx, sy, boxPad))
        continue;

      double u;
      double v;
      if (!ff.Face.ClosestPoint(seed, out u, out v))
        continue;
      if (ff.Face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
        continue;

      Point3d p = ff.Face.PointAt(u, v);
      if (!p.IsValid)
        continue;

      Vector3d n = ff.Face.NormalAt(u, v);
      if (!n.Unitize())
        continue;

      double dot = Math.Abs(n * plane.ZAxis);
      if (dot < normalDotMin)
        continue;

      double px;
      double py;
      plane.ClosestParameter(p, out px, out py);

      double dx = px - sx;
      double dy = py - sy;
      double planarDistance = Math.Sqrt(dx * dx + dy * dy);

      if (planarDistance > maxPlanarSnap)
        continue;

      double heightDistance = Math.Abs(PlaneHeight(p, plane) - PlaneHeight(seed, plane));
      double metric = planarDistance + heightDistance * 0.02;

      if (metric < bestMetric)
      {
        bestMetric = metric;
        hit = p;
        found = true;
      }
    }

    return found;
  }

  private bool TryEstimateHeightRobust(double x, double y, List<SourceSample> samples, double radius, double scale, out double height)
  {
    height = 0.0;

    if (samples == null || samples.Count == 0)
      return false;

    int k = 30;
    double[] bestD2 = new double[k];
    double[] bestH = new double[k];

    double searchRadius = Math.Max(radius, scale * 0.02);
    int found = 0;

    for (int attempt = 0; attempt < 4; attempt++)
    {
      for (int i = 0; i < k; i++)
      {
        bestD2[i] = double.MaxValue;
        bestH[i] = 0.0;
      }

      found = 0;
      double r2 = searchRadius * searchRadius;

      for (int i = 0; i < samples.Count; i++)
      {
        double dx = samples[i].X - x;
        double dy = samples[i].Y - y;
        double d2 = dx * dx + dy * dy;

        if (d2 > r2)
          continue;
        if (d2 >= bestD2[k - 1])
          continue;

        int insert = k - 1;
        while (insert > 0 && d2 < bestD2[insert - 1])
        {
          bestD2[insert] = bestD2[insert - 1];
          bestH[insert] = bestH[insert - 1];
          insert--;
        }

        bestD2[insert] = d2;
        bestH[insert] = samples[i].H;
      }

      for (int i = 0; i < k; i++)
      {
        if (bestD2[i] < double.MaxValue)
          found++;
      }

      if (found >= 6)
        break;

      searchRadius *= 1.75;
    }

    if (found < 1)
      return false;

    List<double> hList = new List<double>();
    for (int i = 0; i < k; i++)
    {
      if (bestD2[i] < double.MaxValue)
        hList.Add(bestH[i]);
    }

    double median = Quantile(hList, 0.50);
    List<double> absDev = new List<double>();

    for (int i = 0; i < hList.Count; i++)
      absDev.Add(Math.Abs(hList[i] - median));

    double mad = Quantile(absDev, 0.50);
    double hThreshold = Math.Max(scale * 0.00025, mad * 4.0);
    double eps = Math.Max(1e-12, searchRadius * searchRadius * 1e-12);

    double sum = 0.0;
    double weight = 0.0;
    int used = 0;

    for (int i = 0; i < k; i++)
    {
      if (bestD2[i] == double.MaxValue)
        continue;
      if (Math.Abs(bestH[i] - median) > hThreshold && found > 8)
        continue;

      double w = 1.0 / Math.Max(bestD2[i], eps);
      sum += bestH[i] * w;
      weight += w;
      used++;
    }

    if (used < 2 || weight <= 0.0)
    {
      sum = 0.0;
      weight = 0.0;

      for (int i = 0; i < k; i++)
      {
        if (bestD2[i] == double.MaxValue)
          continue;

        double w = 1.0 / Math.Max(bestD2[i], eps);
        sum += bestH[i] * w;
        weight += w;
      }
    }

    if (weight <= 0.0)
      return false;

    height = sum / weight;
    return RhinoMath.IsValidDouble(height);
  }

  private void InpaintMissingInteriorHeights(Point3d[,] baseGrid, Point3d[,] grid, bool[,] fixedMask, Plane plane, int iterations)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);
    iterations = ClampInt(iterations, 20, 2500);

    for (int pass = 0; pass < iterations; pass++)
    {
      double[,] nextHeight = new double[uCount, vCount];

      for (int v = 0; v < vCount; v++)
      {
        for (int u = 0; u < uCount; u++)
          nextHeight[u, v] = PlaneHeight(grid[u, v], plane);
      }

      for (int v = 1; v < vCount - 1; v++)
      {
        for (int u = 1; u < uCount - 1; u++)
        {
          if (fixedMask[u, v])
            continue;

          double sum = 0.0;
          double weight = 0.0;

          AddNeighborHeight(grid, plane, u - 1, v, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u, v - 1, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u, v + 1, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u - 1, v - 1, 0.35, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v - 1, 0.35, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u - 1, v + 1, 0.35, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v + 1, 0.35, ref sum, ref weight);

          if (weight > 0.0)
          {
            double current = PlaneHeight(grid[u, v], plane);
            double avg = sum / weight;
            nextHeight[u, v] = current + (avg - current) * 0.88;
          }
        }
      }

      for (int v = 1; v < vCount - 1; v++)
      {
        for (int u = 1; u < uCount - 1; u++)
        {
          if (!fixedMask[u, v])
            grid[u, v] = SetPointHeight(baseGrid[u, v], nextHeight[u, v], plane);
        }
      }
    }
  }

  private void FairInteriorGridWithHardBoundary(
    Point3d[,] baseGrid,
    Point3d[,] grid,
    Curve[] rails,
    bool[,] anchorMask,
    Plane plane,
    int passes,
    double fairing,
    double anchorStrength)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    passes = ClampInt(passes, 0, 100);
    fairing = ClampDouble(fairing, 0.0, 0.95);
    anchorStrength = ClampDouble(anchorStrength, 0.0, 5.0);

    if (passes <= 0 || fairing <= 0.0)
      return;

    double[,] anchorHeight = new double[uCount, vCount];

    for (int v = 0; v < vCount; v++)
    {
      for (int u = 0; u < uCount; u++)
        anchorHeight[u, v] = PlaneHeight(grid[u, v], plane);
    }

    for (int pass = 0; pass < passes; pass++)
    {
      double[,] nextHeight = new double[uCount, vCount];

      for (int v = 0; v < vCount; v++)
      {
        for (int u = 0; u < uCount; u++)
          nextHeight[u, v] = PlaneHeight(grid[u, v], plane);
      }

      for (int v = 1; v < vCount - 1; v++)
      {
        double tv = (double) v / (double) (vCount - 1);

        for (int u = 1; u < uCount - 1; u++)
        {
          double tu = (double) u / (double) (uCount - 1);

          double sum = 0.0;
          double weight = 0.0;

          AddNeighborHeight(grid, plane, u - 1, v, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u, v - 1, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u, v + 1, 1.0, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u - 1, v - 1, 0.30, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v - 1, 0.30, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u - 1, v + 1, 0.30, ref sum, ref weight);
          AddNeighborHeight(grid, plane, u + 1, v + 1, 0.30, ref sum, ref weight);

          if (weight <= 0.0)
            continue;

          double current = PlaneHeight(grid[u, v], plane);
          double avg = sum / weight;
          double smoothed = current + (avg - current) * fairing;

          if (anchorMask[u, v])
          {
            double edgeDistance = Math.Min(Math.Min(tu, 1.0 - tu), Math.Min(tv, 1.0 - tv));
            double edgeFactor = SmoothStep(ClampDouble(edgeDistance / 0.18, 0.0, 1.0));
            double localAnchor = anchorStrength * edgeFactor;
            smoothed = (smoothed + anchorHeight[u, v] * localAnchor) / (1.0 + localAnchor);
          }

          nextHeight[u, v] = smoothed;
        }
      }

      for (int v = 1; v < vCount - 1; v++)
      {
        for (int u = 1; u < uCount - 1; u++)
          grid[u, v] = SetPointHeight(baseGrid[u, v], nextHeight[u, v], plane);
      }

      EnforceBoundaryGrid(grid, rails);
    }
  }

  private void AddNeighborHeight(Point3d[,] grid, Plane plane, int u, int v, double w, ref double sum, ref double weight)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    if (u < 0 || v < 0 || u >= uCount || v >= vCount)
      return;
    if (!grid[u, v].IsValid || w <= 0.0)
      return;

    sum += PlaneHeight(grid[u, v], plane) * w;
    weight += w;
  }

  private double ComputeGridRoughness(Point3d[,] grid, Plane plane)
  {
    if (grid == null)
      return 0.0;

    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    double sum = 0.0;
    int count = 0;

    for (int v = 1; v < vCount - 1; v++)
    {
      for (int u = 1; u < uCount - 1; u++)
      {
        double h = PlaneHeight(grid[u, v], plane);
        double hx = 0.5 * (PlaneHeight(grid[u - 1, v], plane) + PlaneHeight(grid[u + 1, v], plane));
        double hy = 0.5 * (PlaneHeight(grid[u, v - 1], plane) + PlaneHeight(grid[u, v + 1], plane));
        double lap = h - 0.5 * (hx + hy);
        sum += Math.Abs(lap);
        count++;
      }
    }

    if (count == 0)
      return 0.0;

    return sum / (double) count;
  }

  // --------------------------------------------------------------------------
  // Network surface creation
  // --------------------------------------------------------------------------

  private void MakeHardBoundaryNetworkCurves(Point3d[,] grid, Curve[] rails, int degreeU, int degreeV, out Curve[] rows, out Curve[] columns)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    List<Curve> rowList = new List<Curve>();
    List<Curve> columnList = new List<Curve>();

    for (int v = 0; v < vCount; v++)
    {
      if (v == 0)
      {
        rowList.Add(rails[0].DuplicateCurve());
        continue;
      }

      if (v == vCount - 1)
      {
        rowList.Add(rails[2].DuplicateCurve());
        continue;
      }

      List<Point3d> pts = new List<Point3d>();
      for (int u = 0; u < uCount; u++)
        pts.Add(grid[u, v]);

      Curve c = CreateInterpolatedCurveSafe(pts, ClampInt(degreeU, 1, 5));
      if (c != null && c.IsValid)
        rowList.Add(c);
    }

    for (int u = 0; u < uCount; u++)
    {
      if (u == 0)
      {
        columnList.Add(rails[3].DuplicateCurve());
        continue;
      }

      if (u == uCount - 1)
      {
        columnList.Add(rails[1].DuplicateCurve());
        continue;
      }

      List<Point3d> pts = new List<Point3d>();
      for (int v = 0; v < vCount; v++)
        pts.Add(grid[u, v]);

      Curve c = CreateInterpolatedCurveSafe(pts, ClampInt(degreeV, 1, 5));
      if (c != null && c.IsValid)
        columnList.Add(c);
    }

    rows = rowList.ToArray();
    columns = columnList.ToArray();
  }

  private NurbsSurface CreateNetworkSurfaceFromCurves(Curve[] rows, Curve[] columns, double edgeTol, double interiorTol, double angleTol, out int error)
  {
    error = 0;

    if (rows == null || columns == null || rows.Length < 2 || columns.Length < 2)
      return null;

    NurbsSurface network = null;

    try
    {
      network = NurbsSurface.CreateNetworkSurface(rows, 0, 0, columns, 0, 0, edgeTol, interiorTol, angleTol, out error);
    }
    catch
    {
      error = -999;
      network = null;
    }

    if (network != null && network.IsValid)
    {
      network.SetDomain(0, new Interval(0.0, 1.0));
      network.SetDomain(1, new Interval(0.0, 1.0));
    }

    return network;
  }

  private NurbsSurface CreateThroughPointSurfaceFromGrid(Point3d[,] grid, int uCount, int vCount, int degreeU, int degreeV)
  {
    List<Point3d> points = new List<Point3d>(uCount * vCount);

    for (int v = 0; v < vCount; v++)
    {
      for (int u = 0; u < uCount; u++)
        points.Add(grid[u, v]);
    }

    NurbsSurface s = null;

    try
    {
      s = NurbsSurface.CreateThroughPoints(points, uCount, vCount, degreeU, degreeV, false, false);
    }
    catch
    {
      s = null;
    }

    if (s != null && s.IsValid)
    {
      s.SetDomain(0, new Interval(0.0, 1.0));
      s.SetDomain(1, new Interval(0.0, 1.0));
    }

    return s;
  }

  private Curve CreateInterpolatedCurveSafe(List<Point3d> pts, int degree)
  {
    if (pts == null)
      return null;

    CullClosePointsInPlace(pts, 1e-9);

    if (pts.Count < 2)
      return null;
    if (pts.Count == 2)
      return new LineCurve(pts[0], pts[1]);

    degree = ClampInt(degree, 1, Math.Min(5, pts.Count - 1));
    Curve c = null;

    try { c = Curve.CreateInterpolatedCurve(pts, degree); }
    catch { c = null; }

    return c;
  }

  // --------------------------------------------------------------------------
  // Deviation
  // --------------------------------------------------------------------------

  private double MeasureOriginalFacesToSurfaceDeviation(List<FitFace> fitFaces, Surface surface, int samples, double normalDotMin, Vector3d panelNormal, out int measuredCount)
  {
    measuredCount = 0;

    if (fitFaces == null || surface == null)
      return double.MaxValue;

    if (!panelNormal.Unitize())
      panelNormal = Vector3d.ZAxis;

    samples = ClampInt(samples, 8, 220);
    double maxDev = 0.0;

    for (int faceIndex = 0; faceIndex < fitFaces.Count; faceIndex++)
    {
      if (fitFaces[faceIndex] == null || fitFaces[faceIndex].Face == null)
        continue;

      BrepFace face = fitFaces[faceIndex].Face;
      Interval du = face.Domain(0);
      Interval dv = face.Domain(1);

      for (int i = 0; i < samples; i++)
      {
        double u = du.ParameterAt((double) i / (double) (samples - 1));

        for (int j = 0; j < samples; j++)
        {
          double v = dv.ParameterAt((double) j / (double) (samples - 1));

          if (face.IsPointOnFace(u, v) == PointFaceRelation.Exterior)
            continue;

          Vector3d n = face.NormalAt(u, v);
          if (!n.Unitize())
            continue;
          if (Math.Abs(n * panelNormal) < normalDotMin)
            continue;

          Point3d p = face.PointAt(u, v);

          double su;
          double sv;
          if (!surface.ClosestPoint(p, out su, out sv))
            continue;

          Point3d q = surface.PointAt(su, sv);
          double d = p.DistanceTo(q);

          if (d > maxDev)
            maxDev = d;

          measuredCount++;
        }
      }
    }

    return maxDev;
  }

  private double MeasureSurfaceBoundaryToRailsDeviation(Surface surface, Curve[] rails, int samples, out int measuredCount)
  {
    measuredCount = 0;

    if (surface == null || rails == null || rails.Length != 4)
      return double.MaxValue;

    for (int i = 0; i < 4; i++)
    {
      if (rails[i] == null || !rails[i].IsValid)
        return double.MaxValue;
    }

    samples = ClampInt(samples, 24, 1500);
    Curve[] surfaceEdges = BuildSurfaceBoundaryPolylines(surface, samples);

    if (surfaceEdges == null || surfaceEdges.Length != 4)
      return double.MaxValue;

    double maxDev = 0.0;

    for (int i = 0; i < 4; i++)
    {
      if (surfaceEdges[i] == null || !surfaceEdges[i].IsValid)
        return double.MaxValue;

      AccumulateCurveToCurveDeviation(rails[i], surfaceEdges[i], samples, ref maxDev, ref measuredCount);
      AccumulateCurveToCurveDeviation(surfaceEdges[i], rails[i], samples, ref maxDev, ref measuredCount);
    }

    return measuredCount > 0 ? maxDev : double.MaxValue;
  }

  private Curve[] BuildSurfaceBoundaryPolylines(Surface surface, int samples)
  {
    if (surface == null)
      return null;

    samples = ClampInt(samples, 24, 1500);

    Interval du = surface.Domain(0);
    Interval dv = surface.Domain(1);

    Polyline bottom = new Polyline();
    Polyline right = new Polyline();
    Polyline top = new Polyline();
    Polyline left = new Polyline();

    for (int i = 0; i < samples; i++)
    {
      double f = (double) i / (double) (samples - 1);
      double u = du.ParameterAt(f);
      double v = dv.ParameterAt(f);

      bottom.Add(surface.PointAt(u, dv.T0));
      right.Add(surface.PointAt(du.T1, v));
      top.Add(surface.PointAt(u, dv.T1));
      left.Add(surface.PointAt(du.T0, v));
    }

    Curve[] result = new Curve[4];
    result[0] = new PolylineCurve(bottom);
    result[1] = new PolylineCurve(right);
    result[2] = new PolylineCurve(top);
    result[3] = new PolylineCurve(left);
    return result;
  }

  private void AccumulateCurveToCurveDeviation(Curve source, Curve target, int samples, ref double maxDev, ref int measuredCount)
  {
    if (source == null || target == null || !source.IsValid || !target.IsValid)
      return;

    samples = ClampInt(samples, 8, 1500);

    for (int i = 0; i < samples; i++)
    {
      double f = (double) i / (double) (samples - 1);
      Point3d p = PointAtNormalizedLengthSafe(source, f);

      double t;
      if (!target.ClosestPoint(p, out t))
        continue;

      Point3d q = target.PointAt(t);
      double d = p.DistanceTo(q);

      if (d > maxDev)
        maxDev = d;

      measuredCount++;
    }
  }

  // --------------------------------------------------------------------------
  // Grid safety/debug
  // --------------------------------------------------------------------------

  private bool ProjectedGridIsNotFolded(Point3d[,] grid, Plane plane, double diag, out string reason)
  {
    reason = "";

    if (grid == null)
    {
      reason = "Grid is null.";
      return false;
    }

    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    double eps = Math.Max(1e-12, diag * diag * 1e-12);
    double referenceSign = 0.0;
    int flips = 0;
    int tiny = 0;
    int checkedCount = 0;

    for (int v = 0; v < vCount - 1; v++)
    {
      for (int u = 0; u < uCount - 1; u++)
      {
        double area = ProjectedQuadArea(grid[u, v], grid[u + 1, v], grid[u + 1, v + 1], grid[u, v + 1], plane);

        if (Math.Abs(area) <= eps)
        {
          tiny++;
          continue;
        }

        double sign = area > 0.0 ? 1.0 : -1.0;

        if (referenceSign == 0.0)
          referenceSign = sign;
        else if (sign != referenceSign)
          flips++;

        checkedCount++;
      }
    }

    if (checkedCount < 4)
    {
      reason = "Too few non-degenerate projected grid cells.";
      return false;
    }

    if (flips > 0)
    {
      reason = "Projected grid contains " + flips + " flipped cells.";
      return false;
    }

    double tinyRatio = (double) tiny / (double) Math.Max(1, (uCount - 1) * (vCount - 1));
    if (tinyRatio > 0.20)
    {
      reason = "Projected grid has too many near-zero cells: " + tinyRatio.ToString("0.0%");
      return false;
    }

    return true;
  }

  private void GetProjectedGridFoldStats(Point3d[,] grid, Plane plane, out int flippedCells, out int tinyCells)
  {
    flippedCells = 0;
    tinyCells = 0;

    if (grid == null)
      return;

    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    double referenceSign = 0.0;

    for (int v = 0; v < vCount - 1; v++)
    {
      for (int u = 0; u < uCount - 1; u++)
      {
        double area = ProjectedQuadArea(grid[u, v], grid[u + 1, v], grid[u + 1, v + 1], grid[u, v + 1], plane);

        if (Math.Abs(area) <= 1e-12)
        {
          tinyCells++;
          continue;
        }

        double sign = area > 0.0 ? 1.0 : -1.0;

        if (referenceSign == 0.0)
          referenceSign = sign;
        else if (sign != referenceSign)
          flippedCells++;
      }
    }
  }

  private double ProjectedQuadArea(Point3d a, Point3d b, Point3d c, Point3d d, Plane plane)
  {
    double ax;
    double ay;
    double bx;
    double by;
    double cx;
    double cy;
    double dx;
    double dy;

    plane.ClosestParameter(a, out ax, out ay);
    plane.ClosestParameter(b, out bx, out by);
    plane.ClosestParameter(c, out cx, out cy);
    plane.ClosestParameter(d, out dx, out dy);

    double area =
      ax * by - ay * bx +
      bx * cy - by * cx +
      cx * dy - cy * dx +
      dx * ay - dy * ax;

    return 0.5 * area;
  }

  private List<Curve> MakeDebugGridCurves(Point3d[,] grid, int maxCurvesPerDirection)
  {
    List<Curve> curves = new List<Curve>();

    if (grid == null)
      return curves;

    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    int uStep = Math.Max(1, uCount / Math.Max(1, maxCurvesPerDirection));
    int vStep = Math.Max(1, vCount / Math.Max(1, maxCurvesPerDirection));

    for (int u = 0; u < uCount; u += uStep)
    {
      Polyline pl = new Polyline();
      for (int v = 0; v < vCount; v++)
        pl.Add(grid[u, v]);
      curves.Add(new PolylineCurve(pl));
    }

    if ((uCount - 1) % uStep != 0)
    {
      Polyline pl = new Polyline();
      for (int v = 0; v < vCount; v++)
        pl.Add(grid[uCount - 1, v]);
      curves.Add(new PolylineCurve(pl));
    }

    for (int v = 0; v < vCount; v += vStep)
    {
      Polyline pl = new Polyline();
      for (int u = 0; u < uCount; u++)
        pl.Add(grid[u, v]);
      curves.Add(new PolylineCurve(pl));
    }

    if ((vCount - 1) % vStep != 0)
    {
      Polyline pl = new Polyline();
      for (int u = 0; u < uCount; u++)
        pl.Add(grid[u, vCount - 1]);
      curves.Add(new PolylineCurve(pl));
    }

    return curves;
  }

  // --------------------------------------------------------------------------
  // Counts and sampling
  // --------------------------------------------------------------------------

  private void EstimateNetworkCounts(Curve[] rails, int degreeU, int degreeV, ref int uCount, ref int vCount)
  {
    double lenU = 0.5 * (SafeCurveLength(rails[0]) + SafeCurveLength(rails[2]));
    double lenV = 0.5 * (SafeCurveLength(rails[1]) + SafeCurveLength(rails[3]));

    int longCount = 24;
    int minCount = 9;

    if (uCount < 4 && vCount < 4)
    {
      if (lenU >= lenV && lenU > RhinoMath.ZeroTolerance)
      {
        uCount = longCount;
        vCount = Math.Max(minCount, (int) Math.Round(longCount * lenV / lenU));
      }
      else if (lenV > RhinoMath.ZeroTolerance)
      {
        vCount = longCount;
        uCount = Math.Max(minCount, (int) Math.Round(longCount * lenU / lenV));
      }
      else
      {
        uCount = longCount;
        vCount = minCount;
      }
    }
    else if (uCount < 4)
    {
      uCount = lenV > RhinoMath.ZeroTolerance ? Math.Max(minCount, (int) Math.Round(vCount * lenU / lenV)) : longCount;
    }
    else if (vCount < 4)
    {
      vCount = lenU > RhinoMath.ZeroTolerance ? Math.Max(minCount, (int) Math.Round(uCount * lenV / lenU)) : minCount;
    }

    uCount = ClampInt(uCount, degreeU + 2, 46);
    vCount = ClampInt(vCount, degreeV + 2, 46);
  }

  private Point3d[] SampleCurve(Curve c, int count)
  {
    Point3d[] pts = new Point3d[count];

    for (int i = 0; i < count; i++)
    {
      double f = (double) i / (double) (count - 1);
      pts[i] = PointAtNormalizedLengthSafe(c, f);
    }

    return pts;
  }

  private Point3d PointAtNormalizedLengthSafe(Curve c, double f)
  {
    f = ClampDouble(f, 0.0, 1.0);
    double len = SafeCurveLength(c);

    if (len > RhinoMath.ZeroTolerance)
    {
      double t;
      if (c.LengthParameter(len * f, out t))
        return c.PointAt(t);
    }

    return c.PointAt(c.Domain.ParameterAt(f));
  }

  private double NormalizedParameter(Curve c, double t)
  {
    Interval d = c.Domain;
    double span = d.T1 - d.T0;

    if (Math.Abs(span) <= RhinoMath.ZeroTolerance)
      return 0.0;

    double n = (t - d.T0) / span;

    while (n < 0.0)
      n += 1.0;
    while (n >= 1.0)
      n -= 1.0;

    return n;
  }

  private double NormalizedCurveLengthAtParameter(Curve c, double t)
  {
    if (c == null || !c.IsValid)
      return 0.0;

    double total = SafeCurveLength(c);
    if (total <= RhinoMath.ZeroTolerance)
      return NormalizedParameter(c, t);

    Interval d = c.Domain;

    if (t <= d.T0)
      return 0.0;
    if (t >= d.T1)
      return 0.0;

    Curve segment = c.Trim(d.T0, t);
    if (segment == null || !segment.IsValid)
      return NormalizedParameter(c, t);

    double len = SafeCurveLength(segment);
    double n = len / total;

    while (n < 0.0)
      n += 1.0;
    while (n >= 1.0)
      n -= 1.0;

    return n;
  }

  private double EstimateProjectedCellSize(Point3d[,] grid, Plane plane)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);

    double sum = 0.0;
    int count = 0;

    for (int v = 0; v < vCount; v++)
    {
      for (int u = 0; u < uCount; u++)
      {
        if (u + 1 < uCount)
        {
          sum += ProjectedDistance(grid[u, v], grid[u + 1, v], plane);
          count++;
        }

        if (v + 1 < vCount)
        {
          sum += ProjectedDistance(grid[u, v], grid[u, v + 1], plane);
          count++;
        }
      }
    }

    if (count < 1)
      return 1.0;

    return sum / (double) count;
  }

  private double ProjectedDistance(Point3d a, Point3d b, Plane plane)
  {
    double ax;
    double ay;
    double bx;
    double by;

    plane.ClosestParameter(a, out ax, out ay);
    plane.ClosestParameter(b, out bx, out by);

    double dx = bx - ax;
    double dy = by - ay;
    return Math.Sqrt(dx * dx + dy * dy);
  }

  private double EstimateSampleScale(List<SourceSample> samples)
  {
    if (samples == null || samples.Count < 2)
      return 1.0;

    double minX = double.MaxValue;
    double maxX = -double.MaxValue;
    double minY = double.MaxValue;
    double maxY = -double.MaxValue;

    for (int i = 0; i < samples.Count; i++)
    {
      if (samples[i].X < minX) minX = samples[i].X;
      if (samples[i].X > maxX) maxX = samples[i].X;
      if (samples[i].Y < minY) minY = samples[i].Y;
      if (samples[i].Y > maxY) maxY = samples[i].Y;
    }

    double dx = maxX - minX;
    double dy = maxY - minY;
    double scale = Math.Sqrt(dx * dx + dy * dy);

    if (!RhinoMath.IsValidDouble(scale) || scale <= RhinoMath.ZeroTolerance)
      return 1.0;

    return scale;
  }

  // --------------------------------------------------------------------------
  // Geometry utilities
  // --------------------------------------------------------------------------

  private double PlaneHeight(Point3d p, Plane plane)
  {
    return (p - plane.Origin) * plane.ZAxis;
  }

  private Point3d PlanePoint(Plane plane, double x, double y, double h)
  {
    return plane.Origin + plane.XAxis * x + plane.YAxis * y + plane.ZAxis * h;
  }

  private Point3d SetPointHeight(Point3d pointWithXY, double height, Plane plane)
  {
    double x;
    double y;
    plane.ClosestParameter(pointWithXY, out x, out y);
    return PlanePoint(plane, x, y, height);
  }

  private Point3d Average(Point3d a, Point3d b)
  {
    return new Point3d(0.5 * (a.X + b.X), 0.5 * (a.Y + b.Y), 0.5 * (a.Z + b.Z));
  }

  private Point3d Blend(Point3d a, Point3d b, double t)
  {
    t = ClampDouble(t, 0.0, 1.0);
    return new Point3d(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
  }

  private double SmoothStep(double t)
  {
    t = ClampDouble(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  private void SmoothOpenPointListInPlace(List<Point3d> pts, double amount)
  {
    if (pts == null || pts.Count < 5)
      return;

    amount = ClampDouble(amount, 0.0, 1.0);
    int passes = ClampInt((int) Math.Round(1.0 + amount * 10.0), 1, 10);
    double alpha = ClampDouble(0.10 + amount * 0.38, 0.0, 0.60);

    for (int pass = 0; pass < passes; pass++)
    {
      List<Point3d> old = new List<Point3d>(pts);

      for (int i = 1; i < pts.Count - 1; i++)
      {
        Point3d avg = Average(old[i - 1], old[i + 1]);
        pts[i] = Blend(old[i], avg, alpha);
      }
    }
  }

  private Point3d[,] CloneGrid(Point3d[,] grid)
  {
    int uCount = grid.GetLength(0);
    int vCount = grid.GetLength(1);
    Point3d[,] clone = new Point3d[uCount, vCount];

    for (int v = 0; v < vCount; v++)
    {
      for (int u = 0; u < uCount; u++)
        clone[u, v] = grid[u, v];
    }

    return clone;
  }

  private double SafeCurveLength(Curve c)
  {
    if (c == null || !c.IsValid)
      return 0.0;

    double len = c.GetLength();
    if (!RhinoMath.IsValidDouble(len))
      return 0.0;

    return len;
  }

  private void CullClosePointsInPlace(List<Point3d> pts, double tol)
  {
    if (pts == null || pts.Count < 2)
      return;

    double t2 = tol * tol;

    for (int i = pts.Count - 2; i >= 0; i--)
    {
      if (pts[i].DistanceToSquared(pts[i + 1]) <= t2)
        pts.RemoveAt(i + 1);
    }
  }

  private double Quantile(List<double> values, double q)
  {
    if (values == null || values.Count == 0)
      return 0.0;

    List<double> copy = new List<double>(values);
    copy.Sort();
    q = ClampDouble(q, 0.0, 1.0);

    if (copy.Count == 1)
      return copy[0];

    double pos = q * (double) (copy.Count - 1);
    int i0 = (int) Math.Floor(pos);
    int i1 = (int) Math.Ceiling(pos);

    if (i0 == i1)
      return copy[i0];

    double t = pos - (double) i0;
    return copy[i0] + (copy[i1] - copy[i0]) * t;
  }

  private int ClampInt(int value, int min, int max)
  {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  private double ClampDouble(double value, double min, double max)
  {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  // --------------------------------------------------------------------------
  // Data containers
  // --------------------------------------------------------------------------

  private class FitFace
  {
    public BrepFace Face;
    public ProjectedBox Box;
    public int FaceIndex;
    public double NormalDot;
    public double ProjectedArea;
  }

  private class SourceSample
  {
    public Point3d Point;
    public double X;
    public double Y;
    public double H;
    public int FaceIndex;
  }

  private class CornerParam
  {
    public double T;
    public double N;
  }

  private class ProjectedBox
  {
    public double MinX = double.MaxValue;
    public double MaxX = -double.MaxValue;
    public double MinY = double.MaxValue;
    public double MaxY = -double.MaxValue;

    public void Include(double x, double y)
    {
      if (x < MinX) MinX = x;
      if (x > MaxX) MaxX = x;
      if (y < MinY) MinY = y;
      if (y > MaxY) MaxY = y;
    }

    public bool IsValid()
    {
      return
        RhinoMath.IsValidDouble(MinX) &&
        RhinoMath.IsValidDouble(MaxX) &&
        RhinoMath.IsValidDouble(MinY) &&
        RhinoMath.IsValidDouble(MaxY) &&
        MaxX > MinX &&
        MaxY > MinY;
    }

    public bool Contains(double x, double y, double pad)
    {
      if (!IsValid())
        return false;

      return
        x >= MinX - pad &&
        x <= MaxX + pad &&
        y >= MinY - pad &&
        y <= MaxY + pad;
    }
  }
}
