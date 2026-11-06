#!/usr/bin/env bash
# つくばworldのメッシュを kyo0221/rsf_simulator (Git LFS) から models/tsukuba/meshes へ取得する。
# 取得後は colcon build --symlink-install --packages-select orne_box_simulation を実行すること。
set -euo pipefail

commit=9a720a029f80ce1dcf7584b7e784c11ac3e6311e
base_url="https://media.githubusercontent.com/media/kyo0221/rsf_simulator/${commit}/models/tsukuba/meshes"
mesh_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/models/tsukuba/meshes"

mkdir -p "${mesh_dir}"
cd "${mesh_dir}"

while read -r sha256 name; do
  if echo "${sha256}  ${name}" | sha256sum --check --status 2>/dev/null; then
    echo "skip: ${name}"
    continue
  fi
  echo "download: ${name}"
  curl -fL --retry 3 -o "${name}.part" "${base_url}/${name}"
  echo "${sha256}  ${name}.part" | sha256sum --check --quiet
  mv "${name}.part" "${name}"
done <<'SHA256'
5fcc495f982421df2c97574fd14faa8830ef4cb3dba27d871f9a5232448e740c terrain.obj
2056b9f51962d2b91450359db75912ea9dbf98d06350b66d5489846947b9b1b5 structures.obj
c4f29c5507f3c438235a655e1640f1667792f81926d3da5713aa0b91800a6e94 terrain_collision.obj
2b2d42148ed0bba40f6f453db3d34a2b4db9f9c40ab4497fb75c654152eb5a88 structures_collision.obj
SHA256
