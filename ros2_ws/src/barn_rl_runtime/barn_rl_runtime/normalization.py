"""Observation normalization using stored running statistics.

STUB. Loads mean/std captured during training (models/normalization/obs_stats.json)
and applies (x - mean) / std. The same statistics MUST be used at train and
inference time.
"""

import json


class Normalizer:
    """Applies a fixed affine normalization to observation vectors."""

    def __init__(self, mean=None, std=None):
        """Store mean/std vectors; identity normalization if both are None."""
        self._mean = mean
        self._std = std

    @classmethod
    def from_json(cls, path):
        """Load mean/std from a JSON file with keys 'mean' and 'std'."""
        with open(path, 'r', encoding='utf-8') as handle:
            stats = json.load(handle)
        return cls(stats.get('mean'), stats.get('std'))

    def apply(self, observation):
        """Return the normalized observation (identity if no stats loaded)."""
        if self._mean is None or self._std is None:
            return list(observation)
        return [
            (x - m) / s if s else (x - m)
            for x, m, s in zip(observation, self._mean, self._std)
        ]
