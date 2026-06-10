# 1 ft Grid Map

The environment is discretized into `140 x 85` cells at `1 ft` resolution.

- Columns: `0..139`
- Rows: `0..84`
- Occupied cells: table footprints
- Free cells: aisles and cross-aisles

## Table Row Bands

- Row 1 tables occupy `y = 10..14 ft`
- Row 2 tables occupy `y = 25..29 ft`
- Row 3 tables occupy `y = 40..44 ft`
- Row 4 tables occupy `y = 55..59 ft`
- Row 5 tables occupy `y = 70..74 ft`

## Table Column Bands Per Row

- Table 1 occupies `x = 0..19 ft`
- Table 2 occupies `x = 30..49 ft`
- Table 3 occupies `x = 60..79 ft`
- Table 4 occupies `x = 90..109 ft`
- Table 5 occupies `x = 120..139 ft`

## Cross-Aisles

- South cross-aisle: `y = 0..9 ft`
- North cross-aisle: `y = 75..84 ft`

## Notes

- The finalized geometry is `85 ft` tall once all five rows, row aisles, and north/south cross-aisles are included.
- The canonical source of truth is `layout_spec.yaml`, with table and goal placements in the CSV files.
