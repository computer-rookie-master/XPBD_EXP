# 分析 `SphereMesh` 代码的底层逻辑与数学公式

## 1.1 球体的几何表示

球体是一个三维几何体，可以通过**球坐标系（Spherical Coordinates）**来表示。一个点在球坐标系中的位置由三个参数定义：

- $r$: 半径（radius），即点到球心的距离。
- $\theta$: 纬度角（stack angle），从北极（$\theta = 0$）到南极（$\theta = \pi$），但在代码中通常调整为从 $\pi/2$（顶部）到 $-\pi/2$（底部）。
- $\phi$: 经度角（sector angle），从 0 到 $2\pi$，绕着球体的赤道平面旋转。

球坐标系到笛卡尔坐标系（Cartesian Coordinates）的转换公式为：

$$
x = r \cdot \cos(\theta) \cdot \cos(\phi)
$$

$$
y = r \cdot \cos(\theta) \cdot \sin(\phi)
$$

$$
z = r \cdot \sin(\theta)
$$

其中：
- $x, y, z$: 点的笛卡尔坐标。
- $r$: 球的半径（在代码中为 `radius`）。
- $\theta$: 纬度角（对应 `stackAngle`）。
- $\phi$: 经度角（对应 `sectorAngle`）。

## 1.2 代码分析与数学逻辑

以下是 `SphereMesh` 构造函数的代码：

**`src/render/sphere_mesh.cpp`**

```cpp
SphereMesh::SphereMesh(float radius, int sectors, int stacks) {
    float sectorStep = 2 * 3.14159f / sectors;
    float stackStep = 3.14159f / stacks;

    // 生成顶点和法线
    for (int i = 0; i <= stacks; ++i) {
        float stackAngle = 3.14159f / 2 - i * stackStep;
        float xy = radius * cosf(stackAngle);
        float z = radius * sinf(stackAngle);

        for (int j = 0; j <= sectors; ++j) {
            float sectorAngle = j * sectorStep;
            float x = xy * cosf(sectorAngle);
            float y = xy * sinf(sectorAngle);

            positions.push_back(glm::vec3(x, y, z));
            normals.push_back(glm::normalize(glm::vec3(x, y, z)));

            // 调试输出：打印部分顶点坐标
            if (i == 0 || i == stacks || j == 0 || j == sectors) {
                std::cout << "Vertex (" << i << ", " << j << "): (" 
                          << x << ", " << y << ", " << z << ")" << std::endl;
            }
        }
    }

    // 生成索引
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < sectors; ++j) {
            int k1 = i * (sectors + 1) + j;
            int k2 = k1 + 1;
            int k3 = (i + 1) * (sectors + 1) + j;
            int k4 = k3 + 1;

            indices.push_back(k1);
            indices.push_back(k3);
            indices.push_back(k4);

            indices.push_back(k1);
            indices.push_back(k4);
            indices.push_back(k2);
        }
    }

    std::cout << "SphereMesh created with " << positions.size() << " vertices and "
              << indices.size() / 3 << " triangles." << std::endl;
}
```

### 1.2.1 步长计算

```cpp
float sectorStep = 2 * 3.14159f / sectors;
float stackStep = 3.14159f / stacks;
```

- **目的**：将球体的经度和纬度分成离散的步长，以便生成网格。
- **数学逻辑**：
  - `sectorStep`：经度角 $\phi$ 的步长。球体的经度范围是 $[0, 2\pi]$，分成 `sectors` 份，每份的角度是：
    $$
    \text{sectorStep} = \frac{2\pi}{\text{sectors}}
    $$
    例如，`sectors = 50`，则：
    $$
    \text{sectorStep} = \frac{2\pi}{50} \approx 0.1256637 \text{ 弧度}
    $$
  - `stackStep`：纬度角 $\theta$ 的步长。纬度范围是 $[-\pi/2, \pi/2]$，总范围是 $\pi$，分成 `stacks` 份，每份的角度是：
    $$
    \text{stackStep} = \frac{\pi}{\text{stacks}}
    $$
    例如，`stacks = 50`，则：
    $$
    \text{stackStep} = \frac{\pi}{50} \approx 0.0628319 \text{ 弧度}
    $$

### 1.2.2 顶点生成

```cpp
for (int i = 0; i <= stacks; ++i) {
    float stackAngle = 3.14159f / 2 - i * stackStep;
    float xy = radius * cosf(stackAngle);
    float z = radius * sinf(stackAngle);

    for (int j = 0; j <= sectors; ++j) {
        float sectorAngle = j * stackStep;
        float x = xy * cosf(sectorAngle);
        float y = xy * sinf(sectorAngle);

        positions.push_back(glm::vec3(x, y, z));
        normals.push_back(glm::normalize(glm::vec3(x, y, z)));
    }
}
```

#### 外层循环（纬度，`i`）
- `i` 从 0 到 `stacks`，表示纬度上的分段。
- `stackAngle`：纬度角 $\theta$，从 $\pi/2$（北极）到 $-\pi/2$（南极）：
  $$
  \theta = \frac{\pi}{2} - i \cdot \text{stackStep}
  $$
  - 当 `i = 0`，`stackAngle = \pi/2`。
  - 当 `i = stacks`，`stackAngle = \pi/2 - \text{stacks} \cdot (\pi/\text{stacks}) = -\pi/2`。
- `xy`：纬度上的投影半径，即球面在 $xy$ 平面上的投影：
  $$
  \text{xy} = r \cdot \cos(\theta)
  $$
- `z`：高度：
  $$
  z = r \cdot \sin(\theta)
  $$
  - 当 $\theta = \pi/2$，`z = radius * sin(\pi/2) = radius`。
  - 当 $\theta = 0$，`z = radius * sin(0) = 0`。
  - 当 $\theta = -\pi/2$，`z = radius * sin(-\pi/2) = -radius`。

#### 内层循环（经度，`j`）
- `j` 从 0 到 `sectors`，表示经度上的分段。
- `sectorAngle`：经度角 $\phi$，从 0 到 $2\pi$：
  $$
  \phi = j \cdot \text{sectorStep}
  $$
  - 当 `j = 0`，`sectorAngle = 0`。
  - 当 `j = sectors`，`sectorAngle = \text{sectors} \cdot (2\pi/\text{sectors}) = 2\pi`。
- `x` 和 `y`：根据经度角计算球面上的坐标：
  $$
  x = \text{xy} \cdot \cos(\phi)
  $$
  $$
  y = \text{xy} \cdot \sin(\phi)
  $$
  - 当 $\phi = 0$，`x = xy * cos(0) = xy`，`y = xy * sin(0) = 0`。
  - 当 $\phi = \pi/2$，`x = xy * cos(\pi/2) = 0`，`y = xy * sin(\pi/2) = xy`。

#### 法线计算
- 球体的法线是顶点位置相对于球心的方向向量。由于球心在原点，顶点位置 $(x, y, z)$ 本身就是法线方向，归一化后即为法线：
  $$
  \text{normal} = \text{normalize}(x, y, z)
  $$
  例如，顶点 $(\text{radius}, 0, 0)$ 的法线是 $(1, 0, 0)$。

### 1.2.3 索引生成

```cpp
for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < sectors; ++j) {
        int k1 = i * (sectors + 1) + j;
        int k2 = k1 + 1;
        int k3 = (i + 1) * (sectors + 1) + j;
        int k4 = k3 + 1;

        indices.push_back(k1);
        indices.push_back(k3);
        indices.push_back(k4);

        indices.push_back(k1);
        indices.push_back(k4);
        indices.push_back(k2);
    }
}
```

- **目的**：将顶点连接成三角形，形成球体的网格。
- **顶点索引计算**：
  - 顶点是以网格形式排列的，每行有 `sectors + 1` 个顶点（因为 `j` 从 0 到 `sectors`）。
  - 总共有 `stacks + 1` 行（因为 `i` 从 0 到 `stacks`）。
  - 顶点索引公式：
    - 第 $(i, j)$ 个顶点的索引是 $i \cdot (\text{sectors} + 1) + j$.
  - 每个格子由四个顶点组成：
    - `k1 = i * (sectors + 1) + j`：左上角。
    - `k2 = k1 + 1`：右上角。
    - `k3 = (i + 1) * (sectors + 1) + j`：左下角。
    - `k4 = k3 + 1`：右下角。
- **三角形划分**：
  - 每个格子被分成两个三角形：
    - 第一个三角形：`k1-k3-k4`。
    - 第二个三角形：`k1-k4-k2`。
  - 这样可以形成一个四边形网格，覆盖整个球体表面。

---