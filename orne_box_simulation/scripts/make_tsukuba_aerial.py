#!/usr/bin/env python3
"""つくばworldの地面に航空写真を貼るためのファイルを作る。

1. 国土地理院の航空写真タイル (全国最新写真 (シームレス), ズーム 18) を取得し、
   地図座標 (tsukuba2026.pcd と同じ x, y) の範囲 X_MIN..X_MAX, Y_MIN..Y_MAX に合わせた画像
   models/tsukuba/materials/textures/tsukuba_aerial.jpg を作る。
   地図の原点は 緯度 36.082880, 経度 140.076545 (JGD2024, raucha/tc-datasets の tc26-ise-robotics)。
   原点のまわりの局所平面 (東 = x, 北 = y) として変換する (範囲内のずれは 1 m 程度以内)。
   写真は 0.48 m/px 相当なので、PCD 地図の地面点の反射強度 (0.1 m/px) の細かい明暗 (0.5 m より細かい成分) を
   写真の明度に足して細かくする (白線や路面の文字が見える)。色と大まかな明るさは写真のまま。地図点のない所は写真のまま。
2. models/tsukuba/meshes/terrain.obj に、x, y から決まる UV を付けた
   models/tsukuba/meshes/terrain_textured.obj を作る。
画像の出典: 国土地理院 地理院タイル (全国最新写真 (シームレス))
使い方: scripts/download_tsukuba_meshes.sh の後に python3 scripts/make_tsukuba_aerial.py [PCD]
  (PCD の既定: orne_box_navigation_executor/config/maps/tsukuba/tsukuba2026.pcd。無ければ写真だけで作る)
"""
import math
import os
import sys
import time
import urllib.request

import cv2
import numpy as np

LAT0, LON0 = 36.082880, 140.076545
X_MIN, X_MAX, Y_MIN, Y_MAX = -130.0, 560.0, -130.0, 260.0   # terrain.obj: x -126.9..556.1, y -126.7..254.7
RES = 0.1           # 画像の解像度 [m/px]
ZOOM = 18
TILE_URL = 'https://cyberjapandata.gsi.go.jp/xyz/seamlessphoto/{z}/{x}/{y}.jpg'

model_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'tsukuba')
mesh_dir = os.path.join(model_dir, 'meshes')
tex_path = os.path.join(model_dir, 'materials', 'textures', 'tsukuba_aerial.jpg')
cache_dir = os.path.join(os.path.expanduser('~'), '.cache', 'gsi_seamlessphoto')
pcd_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    model_dir, '..', '..', '..', 'orne_box_navigation_executor', 'config', 'maps', 'tsukuba', 'tsukuba2026.pcd')

# GRS80
a = 6378137.0
f = 1 / 298.257222101
e2 = f * (2 - f)
p0 = math.radians(LAT0)
radius_m = a * (1 - e2) / (1 - e2 * math.sin(p0) ** 2) ** 1.5   # 子午線曲率半径
radius_n = a / math.sqrt(1 - e2 * math.sin(p0) ** 2)            # 卯酉線曲率半径


def to_latlon(x, y):
    return LAT0 + np.degrees(y / radius_m), LON0 + np.degrees(x / (radius_n * math.cos(p0)))


def to_tile(lat, lon):
    n = 2 ** ZOOM
    tx = (lon + 180) / 360 * n
    ty = (1 - np.log(np.tan(np.radians(lat)) + 1 / np.cos(np.radians(lat))) / math.pi) / 2 * n
    return tx, ty


def make_texture():
    corners = [to_tile(*to_latlon(x, y)) for x in (X_MIN, X_MAX) for y in (Y_MIN, Y_MAX)]
    tx0 = int(min(c[0] for c in corners))
    tx1 = int(max(c[0] for c in corners))
    ty0 = int(min(c[1] for c in corners))
    ty1 = int(max(c[1] for c in corners))
    os.makedirs(cache_dir, exist_ok=True)
    mosaic = np.zeros(((ty1 - ty0 + 1) * 256, (tx1 - tx0 + 1) * 256, 3), np.uint8)
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            fn = os.path.join(cache_dir, '%d_%d_%d.jpg' % (ZOOM, tx, ty))
            if not os.path.exists(fn):
                req = urllib.request.Request(
                    TILE_URL.format(z=ZOOM, x=tx, y=ty), headers={'User-Agent': 'orne_box_simulation'})
                data = urllib.request.urlopen(req, timeout=20).read()
                with open(fn + '.part', 'wb') as f_:
                    f_.write(data)
                os.replace(fn + '.part', fn)
                time.sleep(0.2)
            tile = cv2.imread(fn)
            mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256, (tx - tx0) * 256:(tx - tx0 + 1) * 256] = tile
    # 画像の上の行 = Y_MAX、左の列 = X_MIN
    xs = X_MIN + (np.arange(int(round((X_MAX - X_MIN) / RES))) + 0.5) * RES
    ys = Y_MAX - (np.arange(int(round((Y_MAX - Y_MIN) / RES))) + 0.5) * RES
    gx, gy = np.meshgrid(xs, ys)
    lat, lon = to_latlon(gx, gy)
    tx, ty = to_tile(lat, lon)
    img = cv2.remap(mosaic, ((tx - tx0) * 256).astype(np.float32), ((ty - ty0) * 256).astype(np.float32),
                    cv2.INTER_CUBIC)
    del gx, gy, lat, lon, tx, ty
    img = sharpen_with_intensity(img)
    os.makedirs(os.path.dirname(tex_path), exist_ok=True)
    cv2.imwrite(tex_path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    print('texture: %s (%dx%d, %.2f m/px)' % (tex_path, img.shape[1], img.shape[0], RES))


def ground_intensity(shape):
    """PCD (x y z intensity, binary float32) の地面点の反射強度を画像と同じ格子に並べる (合計と点数)"""
    with open(pcd_path, 'rb') as f_:
        off = 0
        while True:
            line = f_.readline()
            if not line:
                sys.exit('%s の DATA 行が見つかりません' % pcd_path)
            off += len(line)
            if line.startswith(b'DATA'):
                if b'binary' not in line:
                    sys.exit('%s は binary の PCD ではありません' % pcd_path)
                break
    pts = np.array(np.memmap(pcd_path, dtype=np.float32, mode='r', offset=off).reshape(-1, 4))
    pts = pts[(pts[:, 0] > X_MIN) & (pts[:, 0] < X_MAX) & (pts[:, 1] > Y_MIN) & (pts[:, 1] < Y_MAX)]
    # 地面点 = 0.3 m 四方で一番低い点から 0.08 m 以内
    jx = ((pts[:, 0] - X_MIN) / 0.3).astype(int)
    jy = ((pts[:, 1] - Y_MIN) / 0.3).astype(int)
    zmin = np.full((jy.max() + 1, jx.max() + 1), np.inf, np.float32)
    np.minimum.at(zmin, (jy, jx), pts[:, 2])
    pts = pts[pts[:, 2] < zmin[jy, jx] + 0.08]
    ix = np.clip(((pts[:, 0] - X_MIN) / RES).astype(int), 0, shape[1] - 1)
    iy = np.clip(((Y_MAX - pts[:, 1]) / RES).astype(int), 0, shape[0] - 1)   # 上の行 = Y_MAX
    s = np.zeros(shape, np.float32)
    c = np.zeros(shape, np.float32)
    np.add.at(s, (iy, ix), pts[:, 3])
    np.add.at(c, (iy, ix), 1)
    return s, c


def sharpen_with_intensity(img):
    """反射強度の細かい明暗を写真の明度 (Lab の L) に足す"""
    if not os.path.exists(pcd_path):
        print('%s がないので写真だけで作ります' % pcd_path)
        return img
    s, c = ground_intensity(img.shape[:2])
    # 点のすき間を埋める (正規化した畳み込み)
    s = cv2.GaussianBlur(s, (0, 0), 1.3)
    c = cv2.GaussianBlur(c, (0, 0), 1.3)
    conf = np.clip(c / 0.25, 0, 1)          # 点の密度から決める確からしさ
    inten = np.log1p(np.where(c > 1e-3, s / np.maximum(c, 1e-6), 0)).astype(np.float32)
    # ごま塩状の点を消す
    top = math.log1p(80.0)
    med = cv2.medianBlur(np.clip(inten / top * 255, 0, 255).astype(np.uint8), 5).astype(np.float32) / 255 * top
    inten = np.where(conf > 0.5, med, inten)
    # 0.5 m (写真の解像度) より細かい成分。点のない所を含めないように重み付きでぼかす
    low = cv2.GaussianBlur(inten * conf, (0, 0), 5.0) / np.maximum(cv2.GaussianBlur(conf, (0, 0), 5.0), 1e-6)
    detail = np.clip(45.0 * (inten - low), -20, 45) * cv2.GaussianBlur(conf, (0, 0), 3.0)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = np.clip(lab[:, :, 0].astype(np.float32) + detail, 0, 255).astype(np.uint8)
    print('sharpened with %s' % pcd_path)
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


def make_textured_obj():
    src = os.path.join(mesh_dir, 'terrain.obj')
    dst = os.path.join(mesh_dir, 'terrain_textured.obj')
    if not os.path.exists(src):
        sys.exit('%s がありません。先に scripts/download_tsukuba_meshes.sh を実行してください' % src)
    with open(src) as fi, open(dst + '.part', 'w') as fo:
        vt = []
        faces = False
        for line in fi:
            if line.startswith('v '):
                _, x, y, _ = line.split()
                vt.append('vt %.6f %.6f\n' % ((float(x) - X_MIN) / (X_MAX - X_MIN),
                                              (float(y) - Y_MIN) / (Y_MAX - Y_MIN)))
            elif line.startswith('f '):
                if not faces:
                    fo.writelines(vt)   # UV は頂点と同じ番号
                    faces = True
                # f a//n b//n c//n -> f a/a/n ...
                parts = [p.split('/') for p in line.split()[1:]]
                line = 'f ' + ' '.join('%s/%s/%s' % (p[0], p[0], p[2]) for p in parts) + '\n'
            fo.write(line)
    os.replace(dst + '.part', dst)
    print('mesh: %s (%d vertices)' % (dst, len(vt)))


if __name__ == '__main__':
    make_texture()
    make_textured_obj()
