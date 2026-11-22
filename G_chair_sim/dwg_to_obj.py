"""
将DWG图纸转换为3D OBJ模型，供Unity导入使用
使用aspose-cad读取DWG文件并导出3D几何体
"""
import os
import sys
import aspose.cad as cad

# DWG文件列表
dwg_files = [
    (r'C:\Users\chiehyu\Desktop\G_chair\斯图尔特6轴平台\第三步-硬件篇\图纸\座椅固定板-大板.dwg',
     r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\座椅固定板-大板.obj'),
    (r'C:\Users\chiehyu\Desktop\G_chair\斯图尔特6轴平台\第三步-硬件篇\图纸\座椅固定板小板.dwg',
     r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\座椅固定板小板.obj'),
    (r'C:\Users\chiehyu\Desktop\G_chair\斯图尔特6轴平台\第三步-硬件篇\图纸\电缸固定板.dwg',
     r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\电缸固定板.obj'),
    (r'C:\Users\chiehyu\Desktop\G_chair\斯图尔特6轴平台\第三步-硬件篇\图纸\鱼眼固定板50X180X5mm厚度铝板6块.dwg',
     r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\鱼眼固定板.obj'),
]

print("=== DWG to 3D OBJ Converter ===")

for src, dst in dwg_files:
    name = os.path.basename(src)
    if not os.path.exists(src):
        print(f"SKIP: {name} - file not found")
        continue

    try:
        print(f"\nConverting: {name}")

        # Load DWG file
        doc = cad.Image.load(src)
        print(f"  Loaded: {doc.width} x {doc.height} pixels")

        # Export as 3D OBJ
        # aspose-cad can export to various formats
        # For 2D DWG drawings, we need to create 3D geometry
        # First, try to export as OBJ (3D format)
        doc.save(dst, cad.ImageFormat.OBJ)
        print(f"  Saved OBJ: {dst} ({os.path.getsize(dst)} bytes)")

        doc.dispose()

    except Exception as e:
        print(f"  ERROR: {e}")
        # Try alternative: export as DXF first, then convert
        try:
            dxf_path = dst.replace('.obj', '.dxf')
            doc = cad.Image.load(src)
            doc.save(dxf_path, cad.ImageFormat.DXF)
            print(f"  Saved DXF as fallback: {dxf_path}")
            doc.dispose()
        except Exception as e2:
            print(f"  DXF fallback also failed: {e2}")

print("\n=== Conversion complete ===")
