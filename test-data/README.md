# Synthetic Star Field Test Data

## Generator

Python scripts to create test scenarios using Hipparcos catalog.

```bash
cd generator
pip install -r requirements.txt
python3 generate_scenarios.py
```

## Scenarios

Binary files containing:
- Star centroids (pixel coordinates)
- Magnitudes
- Ground truth (attitude, star IDs)

## Data Format

See [../docs/protocol/test_data_format.md](../docs/protocol/test_data_format.md)
