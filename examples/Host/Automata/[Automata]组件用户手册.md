# [Automata]组件用户手册

---

## 基本概念和术语

### <span style="color:#2F6BFF;">状态</span>（State）

使用枚举定义<span style="color:#2F6BFF;">状态</span>：

```cpp
enum States { ON, OFF };
```

也可以：

```cpp
enum class States { ON, OFF };
using States::ON, States::OFF;
```

---

### <span style="color:#1E9E5A;">输入</span>（Input）<span style="color:#0F8C8C;">输入量</span>（Item）

支持以下注册方式：

- 结构体成员：`.item<...>(&Struct::member)`
- 普通变量：`.item<...>(variable)`
- 枚举变量：`.item<...>(enum_var)`
- 枚举常量：`.item<...>(Enum::value)`
- 临时对象：`.item<...>(Type{})`

<span style="color:#1E9E5A;">输入</span>按照注册顺序自动编号（从 0 开始）。

---

### <span style="color:#D97706;">组件</span>（Component）

由 item 注册的<span style="color:#0F8C8C;">输入量</span>会被封装为<span style="color:#D97706;">组件</span>。

<span style="color:#D97706;">组件</span>是依据一些规则，可由用户自定义的，能够按自定义方式与<span style="color:#0F8C8C;">输入量</span>交互的类。

每个<span style="color:#D97706;">组件</span>代表一个通道/<span style="color:#0F8C8C;">输入量</span>/变量。

默认<span style="color:#D97706;">组件</span>：

`control::AutomataInputEdge`
`control::AutomataInputRaw`

注册时定义：

```cpp
.item<control::AutomataInputEdge>(...) 	//索引0
.item<control::AutomataInputRaw>(...)	//索引1
```

要在<span style="color:#C2410C;">状态转移</span>逻辑定义中使用：

```cpp
COMPONENT(index)
```

示例：

```cpp
auto& comp = COMPONENT(0);
comp.downEdge(); // 自定义交互方式，当输入量下降时返回true
```

---

### <span style="color:#C2410C;">状态转移</span>（Transition）

定义方式：

```cpp
.transition<STATE_BEGINS_WITH, STATE_GOES_TO>(TRANLOGIC {
    return a_bool_condition;
})
```

逻辑也可以单独定义，以便复用：

```cpp
auto transitionLogic = TRANLOGIC {...};
.transition<STATE_BEGINS_WITH, STATE_GOES_TO>(transitionLogic)
```

特点：

- 返回 bool
- 按定义顺序匹配（先写优先级更高）

注意：逻辑定义不能直接使用局部变量，需要预先注册。转移函数可以直接使用在全局定义的符号。
设计良好的<span style="color:#D97706;">组件</span>能独立的使用，全局符号也可以这样来获得一致的接口，但需要用户自己管理<span style="color:#1E9E5A;">输入</span>。

---

### <span style="color:#7C3AED;">标签</span>（Tag）

<span style="color:#7C3AED;">标签</span>是一套用于直接修改转置逻辑行为的系统，方便用户以简洁的语法构建自动机。

目前支持的<span style="color:#7C3AED;">标签</span>：

`control::ForwardTag`
`control::ReverseTag`

**ForwardTag（默认）**
正常执行逻辑。

**ReverseTag**
反转逻辑，即当`logic == false`时发生转移。

```cpp
.transition<...>(..., control::ReverseTag{})
```

---

## 构建自动机

### 基本结构

```cpp
auto aut = control::AutomataBuilder<EnumStatesCollection>()
    .item<Component>(item)
    .transition<STATE_BEGINS_WITH, STATE_GOES_TO>(transitionLogic, Tag)
    .build<INITIAL_STATE>();
```

**示例1**

```cpp
auto aut = control::AutomataBuilder<States>()
    .item<control::AutomataInputEdge>(&raw_data_struct::num1)
    .transition<ON, OFF>(TRANLOGIC {
        auto& comp = COMPONENT(0);
        return comp.downEdge();
    })
    .transition<OFF, ON>(TRANLOGIC { return rand() % 10 <= 8; }, control::ReverseTag{})
    .build<ON>();
```

**示例2**

```cpp
// 射击（供弹轮）
auto shoot_builder_with_items = control::AutomataBuilder<ShootMode>()
    .item<control::AutomataInputEdge>(dbus->swl)
    .item<control::AutomataInputRaw>(SHOOT_FRIC_MODE_STOP)
    .item<control::AutomataInputRaw>(is_referee_shoot_available)
    .item<control::AutomataInputEdge>(bool{}); // shoot_key

auto tranlogic_shooting_condition = TRANLOGIC {
    const auto& swl = COMPONENT(0);
    const auto& fric_state = COMPONENT(1);
    const auto& referee_permit = COMPONENT(2);
    const auto& shoot_key = COMPONENT(3);
    return (swl.get() == remote::DOWN || shoot_key.get()) &&
        fric_state.get() == SHOOT_FRIC_MODE_PREPARED &&
        referee_permit.get();
};

auto shoot_aut = shoot_builder_with_items
	.transition<SHOOT_MODE_SINGLE, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
    .transition<SHOOT_MODE_BURST, SHOOT_MODE_STOP>(tranlogic_shooting_condition, control::ReverseTag{})
    .transition<SHOOT_MODE_STOP, SHOOT_MODE_SINGLE>(tranlogic_shooting_condition)
    .transition<SHOOT_MODE_SINGLE, SHOOT_MODE_BURST>(TRANLOGIC {
        const auto& swl = COMPONENT(0);
        const auto& shoot_key = COMPONENT(3);
        constexpr uint16_t burst_threshold_cycle = 250;
        return swl.lastUpdate() > burst_threshold_cycle ||
            shoot_key.lastUpdate() > burst_threshold_cycle;
    })
    .build<SHOOT_MODE_STOP>();
```
### 构建方式

**栈分配（推荐）**

```cpp
.build<ON>();
```

**堆分配（不推荐）**

```cpp
.build_heap_allocation<ON>();
```

堆分配需要手动管理生命周期，并带来额外开销。

---

## 使用自动机

### 自动机<span style="color:#1E9E5A;">输入</span>

```cpp
aut.input(std::make_tuple(...));
```

注意：顺序必须与 `.item()` 注册顺序一致。
如果<span style="color:#1E9E5A;">输入</span>类型或<span style="color:#1E9E5A;">输入</span>长度与注册时不一致则编译器报错。

<span style="color:#1E9E5A;">输入</span>更新之后自动机会自动步进一次，如没有任何转置条件成立则默认停留在当前<span style="color:#2F6BFF;">状态</span>。

### 自动机输出

```cpp
aut.state();
```

示例：

```cpp
switch (aut.state()) {
    case ON:
        std::cout << "ON";
        break;
    case OFF:
        std::cout << "OFF";
        break;
}
```

**运行阶段示例**

```cpp
while (true) {
    aut.input(std::make_tuple(raw_data1.num1));
    std::cout << static_cast<int>(aut.state()) << std::endl;
}
```

---

## 设计与使用建议

**拆分系统**
建议将复杂逻辑拆分为多个小自动机，通过将输出作为<span style="color:#1E9E5A;">输入</span>组合。

**<span style="color:#C2410C;">状态转移</span>逻辑**
先定义的 `transition` 优先级更高。
<span style="color:#C2410C;">状态转移</span>逻辑必须返回 `bool`。
可以直接使用全局符号。

**<span style="color:#D97706;">组件</span>使用**
建议仅用于条件判断，避免复杂逻辑和副作用。
<span style="color:#D97706;">组件</span>可以单独使用，不一定要注册到自动机中使用。

**<span style="color:#1E9E5A;">输入</span>**
<span style="color:#1E9E5A;">输入</span>顺序必须与注册顺序严格一致。

**构建**
避免使用堆分配，使用全局变量广播自动机输出是可以接受的设计。

**格式**
推荐对定义automata的块关闭clang-format。
```cpp
// clang-format off

auto ptr = control::AutomataBuilder<states>()
        // ...
        .build_heap_allocation<ON>();
    ptr->input(std::make_tuple());

// clang-format on
```
> I tried, really. But clang-format is what it is ¯\(ツ)/¯.

---
## [高级]自定义<span style="color:#D97706;">组件</span>

自动机系统支持用户自定义<span style="color:#1E9E5A;">输入</span><span style="color:#D97706;">组件</span>，通过 `.item<control::AutomataInputCustom>(...)` 注册到自动机中。

### 语法约束

自定义<span style="color:#D97706;">组件</span>必须满足以下最小接口要求：

- 必须为模板类，包含模板参数 `T`
- 必须提供成员函数：`void update(const T& value)`（或等价语义）

示例骨架：

```cpp
template<typename T>
class MyComponent : public control::AutomataInputComponentsBase<T> {
public:
    void update(const T& value) {
        // 更新内部状态
        curr_ = value;
    }
    // 自定义交互方式
    T plus2AndSquare() const { return std::pow(curr_ + 2, 2); }

private:
    T curr_;
};
```

推荐从基础类继承以复用通用逻辑：

```cpp
template<typename T>
class MyComponent : public control::AutomataInputComponentsBase<T> {
    // 扩展或覆盖行为
};
```
> 虽然不是强制要求，但继承基础类可以获得一致的接口与更好的可维护性。

像默认<span style="color:#D97706;">组件</span>一样注册：

```cpp
.item<control::AutomataInputCustom>(...)
```

注册后，可在 `TRANLOGIC` 中通过 `COMPONENT(index)` 访问对应<span style="color:#D97706;">组件</span>实例。

### 组织与包含建议

- 可以直接将自定义<span style="color:#D97706;">组件</span>写在默认文件 `AutomataInputComponents.h` 中
- 更推荐将自定义<span style="color:#D97706;">组件</span>放入独立头文件中（如 `MyComponents.h`）
- 为了使用方便，可在 `Automata.h` 中统一 `#include` 自定义<span style="color:#D97706;">组件</span>头文件

这样可以保持项目结构清晰、依赖关系简单。

### <span style="color:#D97706;">组件</span>设计注意事项

- 建议适当使用`concept/constexpr/specification`来限制<span style="color:#D97706;">组件</span>的类型。
- 默认<span style="color:#D97706;">组件</span>与自定义<span style="color:#D97706;">组件</span>不一定必须绑定到自动机使用。
- <span style="color:#D97706;">组件</span>设计可以使用虚函数特性，自动机系统理论上不使用继承多态（虚函数表）。
- 设计良好的<span style="color:#D97706;">组件</span>可以独立使用，作为输入处理或信号预处理模块。
​例如：边沿检测、滤波、计时器等<span style="color:#D97706;">组件</span>，都可以在自动机之外复用

---
## [高级]实现细节和性能描述

本自动机系统主要由两个核心部分构成：

- 有限状态机（Finite State Machine FSM）
- 数据管理系统（Data Management System DMS）

系统从设计之初即面向嵌入式环境（如 Cortex-M 系列），
目标是在资源受限条件下提供接近零开销（zero-overhead）的自动机执行性能。

自动机整体采用模板元编程（Template Metaprogramming）构建，其核心思想是：

> **将运行时逻辑尽可能前移到编译期完成**

具体表现为：

自动机的定义过程本质上是类型的逐步构建与累积
大部分结构信息（<span style="color:#2F6BFF;">状态</span>、<span style="color:#C2410C;">状态转移</span>、<span style="color:#D97706;">组件</span>）均编码在类型系统中
运行时仅对已构建的静态结构进行评估

### DMS（数据管理系统）

数据管理系统负责维护所有<span style="color:#1E9E5A;">输入</span><span style="color:#D97706;">组件</span>（Components），其关键设计：

所有<span style="color:#D97706;">组件</span>类型在编译期完全确定
使用 `std::tuple` 连续存储所有<span style="color:#D97706;">组件</span>实例
不涉及动态分配（无 `new` / `malloc`）

**性能特性**

- 内存布局连续，缓存友好
- 编译器可完全展开访问（`get<index>()`）
- 无虚函数、无运行时多态
- 由于完整的类型信息，即便<span style="color:#D97706;">组件</span>使用继承，理论上不使用虚函数表。

### FSM（有限状态机）

FSM 部分负责<span style="color:#C2410C;">状态转移</span>，其关键设计：

使用无捕获 lambda
逻辑作为模板参数存储在 `Transition` 类型中
通过模板展开遍历所有<span style="color:#C2410C;">状态转移</span>

**性能特性**

- 在编译期参与类型构建
- 转移逻辑在调用点直接内联展开
- 无函数指针、无虚调用

### 内联与编译优化

该系统高度依赖编译器优化能力：
所有核心路径均为：

- `constexpr`
- 模板函数
- 无捕获 lambda

在启用优化时（如 `-O1` 及以上）：

- <span style="color:#C2410C;">状态转移</span>判断通常会被完全内联
- `tuple` 访问会被优化为直接内存访问
- 不产生额外调用开销

注意：

> 在 `-O0`（无优化）下，编译器可能不会内联函数。
>
> 基于类型的方案确实会导致实例化更多底层代码，导致程序体积增加（以非易失存储空间和内存空间换取执行效率）。

---

## [高级]系统修改与维护约束

在维护或扩展系统时，必须遵守以下原则：

**禁止引入运行时多态**

避免：

- `std::function`
- 函数指针作为核心路径

**避免动态行为**

避免：

- 动态分配（`new`, `malloc`）
- 动态容器（如 `std::vector` 在核心路径）

**保持类型驱动设计**

- 新功能应优先考虑模板参数表达
- 逻辑应尽量编码在类型系统中

**警惕“隐式性能退化”**

任何引入运行时分支或间接调用的改动，都可能导致：

- 内联失败
- 指令数量增加
- 流水线空泡，性能显著下降（在 MCU 上尤为明显）

---

