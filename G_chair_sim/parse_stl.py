"""
解析 STL 文件（支持 ASCII 和 Binary），输出 JSON 网格数据供 Unity 使用
每个 STL 文件输出：vertices (x,y,z 列表), normals, triangles (3顶点索引)
"""
import struct
import json
import os
import sys

def parse_stl(filepath):
    """解析 STL 文件，自动检测 ASCII/Binary 格式"""
    with open(filepath, 'rb') as f:
        data = f.read()

    size = len(data)

    # 尝试 ASCII 格式：检查是否以 "solid" 开头且大部分是可读文本
    if data[:5] == b'solid':
        # 检查是否纯 ASCII（尝试解码前1000字节）
        try:
            data[:1000].decode('ascii')
            # 进一步检查：如果包含大量非打印字符，则是二进制
            non_printable = sum(1 for b in data[:500] if b < 32 and b not in (9, 10, 13))
            if non_printable > 50:
                # 二进制 STL
                return parse_binary_stl(data)
            else:
                # ASCII STL
                return parse_ascii_stl(data)
        except UnicodeDecodeError:
            return parse_binary_stl(data)
    else:
        return parse_binary_stl(data)

def parse_binary_stl(data):
    """解析二进制 STL"""
    if len(data) < 84:
        raise ValueError("File too small for binary STL")

    # 二进制 STL: 80字节头部 + 4字节三角面数 + 每个三角面50字节
    n_triangles = struct.unpack('<I', data[80:84])[0]
    expected_size = 84 + n_triangles * 50

    if len(data) != expected_size:
        # 有些文件有额外数据，尝试解析
        n_triangles = min(n_triangles, (len(data) - 84) // 50)

    vertices = []
    normals = []
    triangles = []

    offset = 84
    for i in range(n_triangles):
        if offset + 50 > len(data):
            break
        # 每个三角面: 12字节法线 + 36字节顶点 + 2字节属性
        nx, ny, nz = struct.unpack('<3f', data[offset:offset+12])
        v1 = struct.unpack('<3f', data[offset+12:offset+24])
        v2 = struct.unpack('<3f', data[offset+24:offset+36])
        v3 = struct.unpack('<3f', data[offset+36:offset+48])

        normals.append([nx, ny, nz])
        base = len(vertices)
        vertices.extend([v1, v2, v3])
        triangles.append([base, base+1, base+2])
        offset += 50

    return {
        'vertices': [[v[0], v[1], v[2]] for v in vertices],
        'normals': normals,
        'triangles': triangles,
        'triangle_count': len(triangles)
    }

def parse_ascii_stl(data):
    """解析 ASCII STL"""
    text = data.decode('ascii', errors='replace')
    vertices = []
    normals = []
    triangles = []

    current_normal = [0.0, 0.0, 0.0]

    for line in text.split('\n'):
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if not parts:
            continue

        if parts[0] == 'facet':
            if parts[1] == 'normal':
                current_normal = [float(parts[2]), float(parts[3]), float(parts[4])]
        elif parts[0] == 'vertex':
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
        elif parts[0] == 'endfacet':
            if len(vertices) >= 3:
                base = len(vertices) - 3
                triangles.append([base, base+1, base+2])
                normals.append(current_normal)

    return {
        'vertices': vertices,
        'normals': normals,
        'triangles': triangles,
        'triangle_count': len(triangles)
    }

def compute_bounds(mesh):
    """计算网格的边界框"""
    if not mesh['vertices']:
        return {'min': [0,0,0], 'max': [0,0,0], 'size': [0,0,0]}
    mins = [min(v[i] for v in mesh['vertices']) for i in range(3)]
    maxs = [max(v[i] for v in mesh['vertices']) for i in range(3)]
    size = [maxs[i] - mins[i] for i in range(3)]
    return {'min': mins, 'max': maxs, 'size': size}

def center_and_normalize(mesh, target_size=0.1):
    """将网格居中并缩放到目标尺寸，单位为米"""
    bounds = compute_bounds(mesh)
    if bounds['size'][0] == 0 and bounds['size'][1] == 0 and bounds['size'][2] == 0:
        return mesh

    center = [
        (bounds['min'][0] + bounds['max'][0]) / 2,
        (bounds['min'][1] + bounds['max'][1]) / 2,
        (bounds['min'][2] + bounds['max'][2]) / 2
    ]

    # 最大尺寸归一化
    max_size = max(bounds['size'])
    if max_size == 0:
        scale = 1.0
    else:
        scale = target_size / max_size

    centered_vertices = []
    for v in mesh['vertices']:
        centered_vertices.append([
            (v[0] - center[0]) * scale,
            (v[1] - center[1]) * scale,
            (v[2] - center[2]) * scale
        ])

    mesh['vertices'] = centered_vertices
    mesh['bounds'] = compute_bounds({'vertices': centered_vertices})
    return mesh

def main():
    stl_files = [
        (r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\鱼眼固定3D打印件.stl',
         r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\STLData\fisheye_mount.json'),
        (r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\丝杆导向滑块3D打印模型.stl',
         r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\STLData\screw_guide.json'),
        (r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\CADModels\120度角码.stl',
         r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\STLData\angle_bracket.json'),
    ]

    output_dir = r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\STLData'
    os.makedirs(output_dir, exist_ok=True)

    print("=== STL Parser ===")
    for src, dst in stl_files:
        name = os.path.basename(src)
        if not os.path.exists(src):
            print(f"SKIP: {name} - not found")
            continue

        print(f"\nParsing: {name}")
        mesh = parse_stl(src)
        mesh = center_and_normalize(mesh)

        bounds = mesh['bounds']
        print(f"  Triangles: {mesh['triangle_count']}")
        print(f"  Vertices: {len(mesh['vertices'])}")
        print(f"  Size: {bounds['size'][0]:.4f} x {bounds['size'][1]:.4f} x {bounds['size'][2]:.4f} m")

        with open(dst, 'w', encoding='utf-8') as f:
            json.dump(mesh, f, separators=(',', ':'))
        print(f"  Saved: {dst} ({os.path.getsize(dst)} bytes)")

    print("\n=== Done ===")

if __name__ == '__main__':
    main()
