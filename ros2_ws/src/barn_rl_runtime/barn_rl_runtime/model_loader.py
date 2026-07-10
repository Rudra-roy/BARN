"""Load and run an exported policy on CPU.

STUB. onnxruntime is imported lazily so the package builds and the node runs
even before a model exists or the runtime is installed. The BARN physical
target is CPU-only (i3-class, no GPU): always benchmark CPU inference latency.
"""


class PolicyModel:
    """Thin wrapper around an ONNX policy for deterministic CPU inference."""

    def __init__(self, model_path=None):
        """Store the model path; the session is created lazily on first infer."""
        self._model_path = model_path
        self._session = None

    def _ensure_session(self):
        """Create the ONNX Runtime CPU session on first use (lazy import)."""
        if self._session is not None or not self._model_path:
            return
        import onnxruntime as ort  # noqa: F401  (lazy, optional dependency)
        self._session = ort.InferenceSession(
            self._model_path, providers=['CPUExecutionProvider'])

    def infer(self, observation):
        """Return a policy action for the observation.

        STUB: returns a zero action ([v=neutral, w=0]) until a model is wired.
        With action space [-1, 1], action[0] = -1 maps to v_min (stationary).
        """
        self._ensure_session()
        if self._session is None:
            return [-1.0, 0.0]
        # STUB: real path builds the input tensor and runs self._session.run(...).
        _ = observation
        return [-1.0, 0.0]
