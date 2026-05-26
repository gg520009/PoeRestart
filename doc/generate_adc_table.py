import math
import openpyxl
from openpyxl.styles import Font, Alignment, PatternFill, Border, Side
from openpyxl.utils import get_column_letter
import os

# Parameters for SDNT1608X103F3380FTF
R25 = 10000.0  # 10k ohms
B = 3380.0     # B-constant
T25 = 298.15   # 25°C in Kelvin

# Circuit parameters
VCC = 3.3       # 3.3V
R_PU = 82000.0  # 82k ohms pull-up
ADC_MAX = 4095  # 12-bit ADC (0 to 4095)

# Calculate resistance
def calc_resistance(temp_c):
    tk = temp_c + 273.15
    r = R25 * math.exp(B * (1.0 / tk - 1.0 / T25))
    return r

# Create workbook
wb = openpyxl.Workbook()
ws = wb.active
ws.title = "NTC ADC Table"

# Grid lines visible
ws.views.sheetView[0].showGridLines = True

# Title block
ws.merge_cells("A1:F1")
ws["A1"] = "SDNT1608X103F3380FTF 热敏电阻 12位ADC采样对照表"
ws["A1"].font = Font(name="Microsoft YaHei", size=14, bold=True, color="FFFFFF")
ws["A1"].alignment = Alignment(horizontal="center", vertical="center")
ws["A1"].fill = PatternFill(start_color="1F4E79", end_color="1F4E79", fill_type="solid")
ws.row_dimensions[1].height = 45

# Spec info block
specs = [
    ("产品型号 (Part Number)", "SDNT1608X103F3380FTF", "供电电压 (VCC)", "3.3 V"),
    ("标称阻值 (R25)", "10 kΩ ± 1%", "上拉电阻 (R_pullup)", "82 kΩ"),
    ("B常数 (B25/50)", "3380 K ± 1%", "ADC 分辨率", "12-bit (0 ~ 4095)"),
    ("工作温度范围", "-55°C ~ +125°C", "采样电路结构", "VCC -> R_pu -> ADC Pin / NTC -> GND"),
]

for idx, (param1, val1, param2, val2) in enumerate(specs, start=2):
    # Parameter 1
    ws.merge_cells(start_row=idx, start_column=1, end_row=idx, end_column=2)
    cell_p1 = ws.cell(row=idx, column=1, value=param1)
    cell_v1 = ws.cell(row=idx, column=3, value=val1)
    
    # Parameter 2
    ws.merge_cells(start_row=idx, start_column=4, end_row=idx, end_column=5)
    cell_p2 = ws.cell(row=idx, column=4, value=param2)
    cell_v2 = ws.cell(row=idx, column=6, value=val2)
    
    # Styling
    font_bold = Font(name="Microsoft YaHei", size=9, bold=True)
    font_regular = Font(name="Microsoft YaHei", size=9)
    fill_gray = PatternFill(start_color="F2F2F2", end_color="F2F2F2", fill_type="solid")
    
    cell_p1.font = font_bold
    cell_p1.fill = fill_gray
    cell_v1.font = font_regular
    cell_p2.font = font_bold
    cell_p2.fill = fill_gray
    cell_v2.font = font_regular
    
    # Alignments
    cell_p1.alignment = Alignment(horizontal="left", vertical="center")
    cell_v1.alignment = Alignment(horizontal="left", vertical="center")
    cell_p2.alignment = Alignment(horizontal="left", vertical="center")
    cell_v2.alignment = Alignment(horizontal="left", vertical="center")
    
    # Borders for spec block
    thin_side = Side(style='thin', color='D9D9D9')
    for c in range(1, 7):
        ws.cell(row=idx, column=c).border = Border(top=thin_side, bottom=thin_side, left=thin_side, right=thin_side)
        
    ws.row_dimensions[idx].height = 20

# Blank row
ws.row_dimensions[6].height = 15

# Table headers
headers = ["温度 (°C)", "NTC电阻 (Ω)", "NTC电阻 (kΩ)", "采样电压 (V)", "ADC十进制值", "ADC十六进制值"]
for col_idx, header in enumerate(headers, start=1):
    cell = ws.cell(row=7, column=col_idx, value=header)
    cell.font = Font(name="Microsoft YaHei", size=10, bold=True, color="FFFFFF")
    cell.fill = PatternFill(start_color="2F5597", end_color="2F5597", fill_type="solid")
    cell.alignment = Alignment(horizontal="center", vertical="center")
    ws.cell(row=7, column=col_idx).border = Border(
        top=Side(style='medium', color='1F4E79'),
        bottom=Side(style='medium', color='1F4E79'),
        left=Side(style='thin', color='D9D9D9'),
        right=Side(style='thin', color='D9D9D9')
    )
ws.row_dimensions[7].height = 25

# Border styles for data cells
thin_border = Border(
    left=Side(style='thin', color='D9D9D9'),
    right=Side(style='thin', color='D9D9D9'),
    top=Side(style='thin', color='D9D9D9'),
    bottom=Side(style='thin', color='D9D9D9')
)

# Fill data
row_idx = 8
for temp_c in range(-55, 126):
    r = calc_resistance(temp_c)
    r_k = r / 1000.0
    
    # Voltage divider formula: V_adc = VCC * R_ntc / (R_pu + R_ntc)
    v_adc = VCC * r / (R_PU + r)
    
    # ADC Code = round(V_adc / VCC * ADC_MAX)
    adc_dec = round(r / (R_PU + r) * ADC_MAX)
    adc_hex = f"0x{adc_dec:03X}"
    
    # Cells
    c_temp = ws.cell(row=row_idx, column=1, value=temp_c)
    c_r = ws.cell(row=row_idx, column=2, value=r)
    c_rk = ws.cell(row=row_idx, column=3, value=r_k)
    c_v = ws.cell(row=row_idx, column=4, value=v_adc)
    c_dec = ws.cell(row=row_idx, column=5, value=adc_dec)
    c_hex = ws.cell(row=row_idx, column=6, value=adc_hex)
    
    # Number formats
    c_temp.number_format = '0'
    c_r.number_format = '#,##0.0'
    c_rk.number_format = '0.000'
    c_v.number_format = '0.0000'
    c_dec.number_format = '0'
    # hex is a string, no format needed
    
    # Alignments
    c_temp.alignment = Alignment(horizontal="center", vertical="center")
    c_r.alignment = Alignment(horizontal="right", vertical="center")
    c_rk.alignment = Alignment(horizontal="right", vertical="center")
    c_v.alignment = Alignment(horizontal="right", vertical="center")
    c_dec.alignment = Alignment(horizontal="right", vertical="center")
    c_hex.alignment = Alignment(horizontal="center", vertical="center")
    
    # Styling and borders
    for col in range(1, 7):
        cell = ws.cell(row=row_idx, column=col)
        cell.font = Font(name="Microsoft YaHei", size=9)
        cell.border = thin_border
        
        # Alternate row coloring (light blue/gray)
        if temp_c % 2 == 0:
            cell.fill = PatternFill(start_color="F2F5F9", end_color="F2F5F9", fill_type="solid")
            
    ws.row_dimensions[row_idx].height = 20
    row_idx += 1

# Autofit column widths
for col in ws.columns:
    max_len = 0
    col_letter = get_column_letter(col[0].column)
    # Check lengths from row 7 downwards (table data)
    for cell in col[6:]:
        if cell.value is not None:
            if isinstance(cell.value, float):
                val_str = f"{cell.value:,.4f}"
            else:
                val_str = str(cell.value)
            max_len = max(max_len, len(val_str))
    ws.column_dimensions[col_letter].width = max(max_len + 8, 15)

# Save the workbook to doc folder
script_dir = os.path.dirname(os.path.abspath(__file__))
output_file = os.path.join(script_dir, "SDNT1608X103F3380FTF_ADC_Table.xlsx")
wb.save(output_file)
print(f"Excel sheet '{output_file}' generated successfully!")
