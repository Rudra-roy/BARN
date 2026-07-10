"""Training-configuration loading."""

import yaml


def load_config(path):
    """Load a YAML training config into a plain dict."""
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle)
