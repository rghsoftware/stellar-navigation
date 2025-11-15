# Synthetic Star Field Test Data

## Generator

Python scripts to create test scenarios using Hipparcos catalog.

```bash
cd generator

# Install dependencies with uv (faster than pip)
uv sync

# Run generator
uv run python generate_scenarios.py
```

## Scenarios

Binary files containing:
- Star centroids (pixel coordinates)
- Magnitudes
- Ground truth (attitude, star IDs)

## Data Format

See [../docs/protocol/test_data_format.md](../docs/protocol/test_data_format.md)
