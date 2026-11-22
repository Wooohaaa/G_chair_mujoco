import json, os

files = ['fisheye_mount.json', 'screw_guide.json', 'angle_bracket.json']
base = r'C:\Users\chiehyu\Desktop\G_chair\GchairUnity\Assets\STLData'

print("=== STL JSON Mesh Data Verification ===")
for f in files:
    path = os.path.join(base, f)
    if os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as fh:
            data = json.load(fh)
        b = data['bounds']
        print("  {}: {} triangles, {} vertices, size={}x{}x{}m".format(
            f, data['triangle_count'], len(data['vertices']),
            b['size'][0], b['size'][1], b['size'][2]))
    else:
        print("  {}: MISSING".format(f))
print("=== Done ===")
