# R1 description provenance record

Research status: 2026-09-19. This record separates facts verified from public
sources from attribution that still requires confirmation from the robot vendor.
It does not treat a product-name match as proof that two model packages are the
same revision.

## Package-local evidence

The package was received as `r1_description`. Its `package.xml` contains:

```xml
<name>r1_description</name>
<version>0.0.0</version>
<description>r1_description</description>
<maintainer email="shaojiale@robotplusplus.com.cn">shaojiale</maintainer>
<license>BSD-3-Clause</license>
```

There is no `<url>` element and no package-local `LICENSE` file. The manifest
therefore identifies a maintainer domain and declares a package license, but it
does not identify the source archive or state the origin and redistribution
terms of each bundled mesh.

## Whole-robot attribution

The vendor's current website says that Hefei Timerover Technology Co., Ltd.
(合肥时空行者科技有限公司) was formerly Shihe Robot (Hefei) Co., Ltd.
(史河机器人（合肥）有限公司), and markets a wheeled dual-arm platform named
行者 R1. It also says that the current R1 is supplied with an official URDF and
Gazebo model:

- https://timerover.com/newsinfo/11198574.html
- https://timerover.com/newsinfo/11249144.html
- https://timerover.com/newsinfo/11250623.html

This supports the company/name connection, but it is not yet a source URL for
this package. The current public R1 description specifies two 6-DoF arms, while
this repository's R1 has two 7-DoF RealMan RM75 arms. A Chinese government
procurement result independently records Shihe Robot (Hefei) as the supplier of
a mobile humanoid-operation platform containing `RM75-B edu`, `RM71D`, and
`CR100` equipment:

- https://www.ccgp.gov.cn/cggg/zygg/zbgg/202506/t20250616_24782603.htm

That is consistent with Shihe integrating RealMan hardware, but it does not
identify this exact full-body URDF or grant redistribution permission for its
body, sensor, gripper, or other vendor meshes. Searches of public GitHub code,
the Timerover site, and indexed web results did not find an exact public copy of
this `r1_description` package as of the research date.

## Verified RealMan RM75 component files

The following local collision meshes are byte-for-byte identical to the RM75
files in RealMan's official ROS repositories. The comparison downloaded the
files from the `humble` branch of `RealManRobot/ros2_rm_robot` at commit
`c941b565e4f9174afa36561f143ef5fbbb744750` and computed SHA-256 locally.
Filename capitalization differs in this package; file content does not.

| Local file | SHA-256 |
| --- | --- |
| `meshes/rm_75_arm/base_link.STL` | `9ef1c6cad6ca4d6484ff9ed9a7d7715b036e2e6100e2bcb8ecd18e2fbc4d04e9` |
| `meshes/rm_75_arm/Link1.STL` | `bbf0716a4a8516a36d198e1fba1696df2b2b354a22bbb8cb7e284ec8f1dca239` |
| `meshes/rm_75_arm/Link2.STL` | `796420ea8922133e8d0bf2a6534f01f4d5bec25729ca7d63b8af3191c67a532f` |
| `meshes/rm_75_arm/Link3.STL` | `a1c7ae0934ea8ae0594a6deb830fc2f9c5dbf0c68444dff55d2f2c3aa5e73092` |
| `meshes/rm_75_arm/Link4.STL` | `9103bd209e04e729b2ef0bd57a93b27310ccc8fdfc9a6d9aeb1ce89866a7994c` |
| `meshes/rm_75_arm/Link5.STL` | `d4ea481e893d4558a1aca714df189f9938ab6b34ec79978fc2520b2dd54bdff9` |
| `meshes/rm_75_arm/Link6.STL` | `6ddd71e96a1ea5c7b2349d2c4bb09de616b27f5a4fa9079c97e86819da551b6e` |
| `meshes/rm_75_arm/Link7_6f.STL` | `f315d056ee0cb52b87b5ae1fe60cd4b3a9a2aadbdac9f91bb784810a5f50a130` |

Official references:

- https://github.com/RealManRobot/ros2_rm_robot/tree/c941b565e4f9174afa36561f143ef5fbbb744750/rm_description/meshes/rm_75_arm
- https://github.com/RealManRobot/rm_robot/tree/3d7063025786c4c5ed2db2ebc09f68d89d589539/rm_description/meshes/RM75_6F

Current official model catalog (the current files are revised and were not used
as the byte-match source above):

- https://github.com/RealManRobot/rm_models/tree/main/RM75

The ROS 1 `RealManRobot/rm_robot` repository contains the same mesh blobs, has a
root Apache-2.0 license, and states that all packages in that repository use
Apache-2.0. The newer `RealManRobot/rm_models` model repository also has a root
Apache-2.0 license. The ROS 2 repository itself has no root license file and its
`rm_description/package.xml` still says `TODO: License declaration`, so the
licensed ROS 1 copy is the clearer redistribution reference for these eight
exact STL files.

The local RM75 visual `.dae` files were not found in those official repositories
and have not been provenance-matched. `Link7.STL`, which is present locally but
is not used by the published standalone R1 URDF variants, also did not match the
current official file. These files remain unresolved together with the rest of
the whole-robot assets.

## Redistribution status and required confirmation

The eight verified RM75 collision STLs have an identifiable Apache-2.0 upstream.
The public evidence collected so far is insufficient to establish redistribution
permission for the complete R1 package and its remaining meshes in a third-party
Apache-2.0 repository. Before proposing the complete robot package upstream,
obtain the following from Timerover/Shihe:

1. the original download or repository URL for this 7-DoF RM75-based R1 revision;
2. a license or written permission covering the full-body URDF, visual meshes,
   collision meshes, textures, sensor models, and gripper assets;
3. confirmation that the `BSD-3-Clause` declaration in `package.xml` covers the
   bundled vendor assets, plus the copyright holder and required notices.

The current Timerover contact page lists telephone/WeChat `159 5692 0668`:
https://timerover.com/lxwm. An older official Timerover article lists
`sales@robotplusplus.com.cn`, which matches the domain in this package's
maintainer address: https://timerover.com/newsinfo/1605340.html.

If that confirmation cannot be obtained, the upstreamable form should omit the
unresolved assets and provide a user-run fetch-and-convert procedure instead.
