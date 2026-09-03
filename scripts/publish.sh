#!/bin/sh
# 把这块板的固件发到公开仓 github.com/DaliangAuto/vision-follow-car-mcu。
#
#   sh scripts/publish.sh [--dry-run]
#
# 形状和 scripts/publish_hs.sh 一模一样（同一套清单、同一套自检），
# 差别只有三处，都在下面注明。驱动板那侧的对应物是
# FOC_Double_C/scripts/publish.sh。
#
# --- 和 publish_hs.sh 的三处差别 ----------------------------------------------
#
# ! 一、REMOTE 是公开的 GitHub 仓，不是公司内网。
#
# ! 二、**publish_hs.sh 自己不发**。它的 REMOTE 指着公司内网的一台机器 ——
#   而那台机器的存在本身就不该出现在公开仓里。
#   ⇒ 发出去的是本脚本（publish.sh），它只提到公开地址。
#   ⚠⚠ 连"解释为什么不发它"的注释里也不许写那个地址。写这份脚本时就栽过一次：
#     注释里把地址原样抄了进来，而这份脚本自己是要发出去的 ——
#     一个用来挡地址外流的文件，成了地址外流的那个文件。
#   ⚠ 这一条是这两个脚本之间**唯一**的内容差异。别的都一样，是故意的：
#     两份清单一旦开始各长各的，就会有一天只有一边知道某个文件该不该出去。
#
# ! 三、带构建脚本，**不带构建产物**。
#   这条推翻了 publish_hs.sh 头部那句「GitHub 上那个 remote 是公开的，
#   构建目录进去没有意义」。那句话是在公开仓还只是"给人读的源码"时写的。
#   ⇒ 一个 clone 下来编不动的固件仓，对拿到板子的人没有用。CubeIDE 把
#     makefile 全生成在 Debug/ 下，不带它，`sh scripts/build.sh` 第一行
#     就找不到 makefile。
#   ⚠ 带的是 makefile / *.mk / objects.list，**任何 .o .elf .bin .hex .map
#     都不带**，最后一步的泄漏检查会把它们拦下来。
#
# --- 下面这些和 publish_hs.sh 相同，理由也相同 --------------------------------
#
# ! 发的是一份**挑选过的文件树**，不是 push 一个分支。本仓历史里有随手进过库
#   的东西（图片、旧 .ioc 备份），而重写历史会改掉每一个提交号，
#   那个号是发布包的身份（dist/*/MANIFEST.txt 的 scm、板子 `=tlm build` 报的数）。
#
# ! 所以公开那份从它自己的第一个提交长起来，每个提交的标题写明它从本仓
#   哪个提交建的，两边对得上。⚠ 台架板报的 scm 在本仓查不到是正常的。
#
# ! 不 force push。远端有本脚本不知道的提交时，推送失败，人去看。
#
# ! 清单是**列出来**的，不是排除。排除清单会把别人后来加的任何东西一起发出去，
#   而那个失败是静默的、单向的。
set -eu

root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"

REMOTE=git@github.com:DaliangAuto/vision-follow-car-mcu.git
WORK="$root/build/publish"          # build/ 在 .gitignore 里，不会进本仓
DRY=0
if [ "${1:-}" = "--dry-run" ]; then
  DRY=1
fi

# --- 什么东西过去 -------------------------------------------------------------
#
# 列出来，不是排除。理由和驱动板一样：排除清单会把**别人后来加的任何东西**
# 一起发出去，而且这个失败是静默的、单向的。
INCLUDE="Core Drivers
         DlaBoard_B.ioc .mxproject makefile.init
         STM32F407VETX_FLASH.ld STM32F407VETX_RAM.ld
         README.md README_CN.md LICENSE"

# ! CubeIDE 的工程文件。内网那份不带它们（服务器上是 Agent 在编，不开 IDE），
#   公开这边带上：拿到板子的人多半是双击工程打开，不是先读 makefile。
#   ⚠ 核过这四份里没有本机路径（E:\ / C:\ / /home/ / 用户名），
#     所以带出去不泄漏这台机器的任何信息。
# ⚠ 用换行分隔，不是空格：其中一个文件名里带空格（"DlaBoard_B Debug.launch"），
#   空格分隔的清单装不下它，而装不下的表现是**静默少发一个文件**。
IDE=".cproject
.project
.settings
DlaBoard_B Debug.launch"

# ⚠⚠ 内部仓的 .gitignore **不发**，发布树自己写一份，见下面的 GITIGNORE。
#
#   2026-09-01 第一次同步就栽在这上面：内部那份里有 `/Debug/`（那条是对着
#   公开 GitHub 仓说的），抄过去之后，`git add -A` 把本脚本特意带上的
#   makefile **全部忽略掉了**。⇒ 服务器上 170 个文件，一个 Debug 都没有。
#
# ⚠ 而同步这一步的检查是绿的 —— 它查的是磁盘上文件在不在，
#   而 git 有没有把它加进去是另一回事。**脚本说 ok ≠ 仓里真的有。**
#   ⇒ 检查那一节现在多问一句 git。

# 仓根的接口文档。⚠ 只发**描述这块板对外说什么话**的那几份。
#
# ! PLAN_ 和 BRIEF_ 不发，理由和驱动板不发 docs/00 一样：它们是带日期的
#   工作台账和会话简报，写给当天拿着烙铁的人，按设计就会过期。半年后读它，
#   读到的是某人当时**打算做什么**，不是这块板**是什么**。
#   ⇒ 台账里某一条要是变成了关于板子的事实，正确的做法是把它挪进下面这几份。
DOCS="AUTONOMOUS_MODE_SPEC.md
      CONTRACT_CAN_控制板_2026-08-27.md
      CONTRACT_串口_控制板_2026-08-30.md"

# ! fix_vultr_codex_path.sh 不发：它是这台机器上的一次性修补，带着服务器路径，
#   对别人只会是误导。
SCRIPTS="build.sh flash.sh gen_build_info.sh gen_release_makefiles.sh release.sh
         publish.sh"

TOOLS="_console.py gen_cmd_table.py make_release.py stamp_image.py"

# 构建系统 + 三个上板工具，见文件开头「关于 /Debug/」那一节。
BUILDSYS="Debug/makefile Debug/objects.mk Debug/sources.mk Debug/objects.list
          Debug/Core/Src/subdir.mk Debug/Core/Startup/subdir.mk
          Debug/Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
          Debug/ISP.py Debug/cli_test.py Debug/drive_test.py"

# --- 生成物一律不发 -----------------------------------------------------------
#
# ! cmd_hash.h 和 build_info.h 在本仓就不入库，理由写在 .gitignore 里：
#   入库之后，改完 foc_can.c 忘了重生成，那个数就是**别的构建**的指纹，
#   而上位机会（正确地）拒绝整张命令表，报的却是一件看起来无关的事。
#   ⇒ 同一条理由对发布树成立。scripts/build.sh 每次自己重生成。
GENERATED="Core/Inc/cmd_hash.h Core/Inc/build_info.h"

# --- 台架板的 uid ------------------------------------------------------------
#
# ! 换成占位符，理由不是保密 —— 一个 MCU 序列号不是秘密。
#   一行「uid 2034…002c 验过」说的是**它被量的那块板**。在别人 clone 下来的
#   仓里读到，它会被读成关于**他那块板**的结论，而句子本身不会说明这一点。
#   ⚠ 今天本仓的文本里一个都没有，这个清单是**为了下一次**：谁贴一段台架
#   输出进文档，最后那道检查会拦住它。
BENCH_UIDS="<台架板 uid> <台架板 uid>"

# --- 树要能被指认 -------------------------------------------------------------
COMMIT=$(git rev-parse --short=7 HEAD)
if [ -n "$(git status --porcelain)" ]; then
  echo "publish: 工作树有未提交的改动。"
  echo "  发出去的东西对不上任何一个提交，而提交标题会写一个不描述它的号。"
  echo "  先提交。"
  git status --short
  exit 1
fi

VER=$(grep -E "^#define FW_VER_(MAJOR|MINOR|PATCH)" Core/Inc/fw_version.h |
      sed -E 's/.*[ \t]([0-9]+)U.*/\1/' | paste -sd. -)

# --- 取服务器上那个仓 ---------------------------------------------------------
if [ ! -d "$WORK/.git" ]; then
  echo "publish: clone $REMOTE"
  rm -rf "$WORK"
  git clone "$REMOTE" "$WORK" 2>/dev/null || {
    # 空仓 clone 会警告且没有分支，那是正常的。
    mkdir -p "$WORK"
    (cd "$WORK" && git init -q -b main && git remote add origin "$REMOTE")
  }
else
  (cd "$WORK" && git fetch -q origin && git reset -q --hard origin/main) || true
fi

# --- 同步 ---------------------------------------------------------------------
export DLA_IDE_FILES="$IDE"
python - "$root" "$WORK" "$SCRIPTS" "$TOOLS" "$DOCS" "$BENCH_UIDS" "$GENERATED" "$BUILDSYS" $INCLUDE <<'PY'
import os
import shutil
import sys

root, work = sys.argv[1], sys.argv[2]
scripts, tools, docs = sys.argv[3].split(), sys.argv[4].split(), sys.argv[5].split()
uids, generated, buildsys = sys.argv[6].split(), sys.argv[7].split(), sys.argv[8].split()
ide = [x for x in os.environ.get("DLA_IDE_FILES", "").splitlines() if x]
wanted, redacted = set(), 0

# ! 文本在出门的路上重写，二进制逐字节拷。uid 只可能出现在文档和抓包里，
#   而对一个目标文件做替换，得到的是一个没人看过的坏文件。
TEXT = (".md", ".txt", ".c", ".h", ".py", ".sh", ".mk", ".json", ".ld", ".s",
        ".ioc", ".yml", ".yaml", ".list", ".init", ".gitignore")


def put(rel):
    global redacted
    src, dst = os.path.join(root, rel), os.path.join(work, rel)
    os.makedirs(os.path.dirname(dst) or work, exist_ok=True)
    if rel.endswith(TEXT) or os.path.basename(rel) in (".gitignore", ".mxproject",
                                                       "makefile", "objects.list"):
        try:
            body = open(src, encoding="utf-8").read()
        except UnicodeDecodeError:
            shutil.copy2(src, dst)
            wanted.add(rel)
            return
        for u in uids:
            if u in body:
                redacted += body.count(u)
                body = body.replace(u, "<台架板 uid>")
        open(dst, "w", encoding="utf-8", newline="").write(body)
    else:
        shutil.copy2(src, dst)
    wanted.add(rel)


SKIP_SUFFIX = (".o", ".d", ".su", ".cyclo", ".map", ".elf", ".bin", ".hex",
               ".axf", ".bak", ".pyc", ".jpg", ".png")


def copy(rel):
    src = os.path.join(root, rel)
    if os.path.isdir(src):
        for base, dirs, files in os.walk(src):
            dirs[:] = [d for d in dirs if d not in ("__pycache__",)]
            for f in files:
                r = os.path.relpath(os.path.join(base, f), root).replace("\\", "/")
                if r in generated or f.endswith(SKIP_SUFFIX):
                    continue
                put(r)
    elif os.path.exists(src):
        put(rel)
    else:
        print("publish: 少了", rel)
        sys.exit(1)


for rel in sys.argv[9:]:
    copy(rel)
for f in ide:
    copy(f)
for f in scripts:
    copy("scripts/" + f)
for f in tools:
    copy("tools/" + f)
for f in docs:
    copy(f)
for f in buildsys:
    copy(f)

# ! 一段前言，放在树根，说一次而不是在二十处各说一句。
#
#   uid 在上面已经换掉了，但 uid 从来不是真正的问题：这些文档里满是在**我们
#   这台车**上量出来的数（档位电平、CAN 计数、刹车缸极性还没验），而它们对
#   没量过的人读起来就像规格。⇒ 说一次，说在一个不可能错过的地方。
PREAMBLE = """# 关于这份源码

**Daliang Auto 整车控制板（`DlaBoard_B`）** 的固件源码。

## 这块板是什么

⚠ **它没有桥，也不转任何电机。** 它是发命令的那一端：踏板、档位、遥控、
上位机、Jetson 都进它，出去的是 CAN 上的行驶请求（驱动板 `FOC_Double_C`
执行）。⇒ 「把桥关掉就安全了」这句话对这块板不成立 —— 它根本没有桥可关。

## ⚠ 文档里的实测数字，是**我们这台车**上的读数

档位开关的电平、CAN 链路计数、遥控通道映射、刹车电缸的极性 ——
全部来自我们自己的车，**不是关于你手上那台的结论，也不是规格**。
⭐ 留着它们是因为**形状比数值有用**。

## 怎么编

```
sh scripts/build.sh                 # Debug
DLA_CONFIG=Release sh scripts/build.sh
DLA_CONFIG=Release sh scripts/release.sh   # 出上位机认得的发布目录
```

⚠ 工具链路径用环境变量盖，不要改脚本：`DLA_TOOLCHAIN_BIN` / `DLA_MAKE_BIN`。
⚠ **发给客户的必须是 Release** —— Debug 构建的板子在 `ver` 里如实报
`debug=1`，上位机每次烧录都会拦一句，而那句话是对的。

## 从哪读起

| 想知道 | 看 |
|---|---|
| 串口上说什么话（含 `id` / `param` / `tlm` / `selftest`） | `CONTRACT_串口_控制板_2026-08-30.md` |
| CAN 上和驱动板怎么约定的 | `CONTRACT_CAN_控制板_2026-08-27.md`（⚠ 权威是 `Core/Inc/foc_can.h`，冲突时以头文件为准）|
| 和 Jetson 的 22 字节帧 | `AUTONOMOUS_MODE_SPEC.md` |
| 参数表、持久化 | `Core/Src/cart_param.c`、`Core/Inc/cart_persist.h` |

⭐ `Core/Inc/` 各头文件顶上那些注释是这份源码最值钱的部分 ——
每一条都对应一次踩坑（"一个报 perm=rw 而执行方仍读编译期常量的参数，
会造出 set 成功、get 读回新值、而机器行为一点没变"）。

## 这份树是怎么来的

由本仓 `scripts/publish.sh` 从内部仓同步，**它自己就是"这里有什么、
为什么"的清单**。⚠ 提交号和内部仓不是同一套，每个提交的标题写着它从
内部哪个提交建的。
"""
open(os.path.join(work, "关于这份源码.md"), "w",
     encoding="utf-8", newline="").write(PREAMBLE)
wanted.add("关于这份源码.md")

# ! 发布树自己的 .gitignore，不是内部仓那一份的副本。
#
#   内部那份里 `/Debug/` 是对着公开 GitHub 仓说的，抄过来会把这个脚本特意
#   带上的 makefile 全部忽略掉（2026-09-01 在内网那份上撞过，一个 Debug 都没有）。
#   它里面另外那些（/selftest/、/shcematicl/、vultr 的路径）在这份树里
#   也不指向任何东西，留着只会让人以为这里排除过它们。
#
# ⇒ 这一份只挡**构建产物**：编译一次就会出现，而它们不该跟着仓走。
#   "这份树里有什么"由 publish.sh 决定，不由 .gitignore 决定。
GITIGNORE = """# 这份树里有什么，由内部仓的 scripts/publish.sh 决定，不由这个文件决定。
# 这里只挡构建产物 —— 编译一次就会出现，而它们不该跟着仓走。
#
# ⚠ Debug/ 下的 makefile 和 *.mk 是**编译必需的**，故意带着，不要加规则排除它们。
*.o
*.d
*.su
*.cyclo
*.map
*.elf
*.bin
*.hex
*.axf
*.list
!Debug/objects.list
Release/
build/
__pycache__/
*.pyc

# 生成物：由 tools/gen_cmd_table.py 和 scripts/gen_build_info.sh 每次重生成。
# ⚠ 入库的话，改完 foc_can.c 忘了重生成，仓里那个数就是**别的构建**的指纹。
Core/Inc/cmd_hash.h
Core/Inc/build_info.h
"""
open(os.path.join(work, ".gitignore"), "w",
     encoding="utf-8", newline="").write(GITIGNORE)
wanted.add(".gitignore")

# 清单上没有的就删掉 —— 这边删一个文件，那边跟着消失，而不是留在那里没人认领。
removed = 0
for base, dirs, files in os.walk(work):
    dirs[:] = [d for d in dirs if d != ".git"]
    for f in files:
        r = os.path.relpath(os.path.join(base, f), work).replace("\\", "/")
        if r not in wanted:
            os.remove(os.path.join(base, f))
            removed += 1

# --- 最后一步：找它本该排除掉的东西 -------------------------------------------
#
# ! 一份白名单**悄悄匹配到了不该匹配的东西**，是值得为它写十行的那种失败，
#   而且它是单向的：推出去就收不回来。
#
# ! 第二条检查是这块板特有的：makefile 里点名要编的每个 .c，树里都得有。
#   ⇒ 别人 clone 下来第一件事是 build，而一个因为少了文件而死的构建，
#   说的是"这份代码坏了"，实际坏的是上面那张清单。
leaked = []
for base, dirs, files in os.walk(work):
    dirs[:] = [d for d in dirs if d != ".git"]
    for f in files:
        p = os.path.join(base, f)
        r = os.path.relpath(p, work).replace("\\", "/")
        if f.endswith((".o", ".elf", ".bin", ".hex", ".map", ".d", ".bak")):
            leaked.append("%s（构建产物）" % r)
            continue
        try:
            body = open(p, encoding="utf-8").read()
        except (UnicodeDecodeError, OSError):
            continue
        for u in uids:
            if u in body:
                leaked.append("%s（台架 uid）" % r)

missing = []
for mk in ("Debug/Core/Src/subdir.mk", "Debug/Core/Startup/subdir.mk",
           "Debug/Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk"):
    for line in open(os.path.join(work, mk), encoding="utf-8"):
        line = line.strip()
        if not line.startswith("../") or not line.split()[0].endswith((".c", ".s")):
            continue
        rel = line.split()[0][3:]
        if not os.path.exists(os.path.join(work, rel)):
            missing.append("%s（%s 要编它）" % (rel, mk))

if leaked or missing:
    print("publish: 拒绝 —— 下面这些不对：")
    for r in (leaked + missing)[:20]:
        print("   ", r)
    sys.exit(1)

print("publish: %d 个文件，删了 %d 个，换掉 %d 处台架 uid，"
      "makefile 点名的源文件全在" % (len(wanted), removed, redacted))
PY

# --- 提交并推送 ---------------------------------------------------------------
cd "$WORK"
git add -A

# ⚠⚠ 问 git，不要问磁盘。
#
#   上面那一节检查的是"文件在不在树里"，而**在树里和进了仓是两回事**：
#   2026-09-01 第一次同步时，抄过来的 .gitignore 里一条 `/Debug/` 把本脚本
#   特意带上的全部 makefile 忽略掉了 —— 同步那一步全绿，服务器上一个都没有。
#
#   白名单是这份树的唯一闸门，所以**任何**被忽略掉的文件都是缺陷，不是设计。
IGNORED=$(git ls-files --others --ignored --exclude-standard | head -20)
if [ -n "$IGNORED" ]; then
  echo "publish: 拒绝 —— 这些同步进了树，而 git 忽略了它们："
  echo "$IGNORED" | sed 's/^/    /'
  echo "  白名单说要发，.gitignore 说别发 —— 两边对上再跑。"
  exit 1
fi

if git diff --cached --quiet; then
  # ⚠⚠ 「没有新改动」不等于「服务器上已经有了」。
  #
  #   上一次推送失败（服务器端 safe.directory 没配、网断、没权限）会留下一个
  #   **提交好但没推出去**的树。这里要是直接退出，下一次运行会说"没有变化"
  #   然后什么都不做 —— 而它每一次都会这么说。⇒ 一个静默的、会一直静默下去的
  #   失败，而症状是别人 clone 到的永远是上一版。
  #
  #   2026-09-01 第一次建这个仓时就撞上了：服务器端 safe.directory 还没配，
  #   commit 成了、push 没成，再跑一次报"没有变化"。
  UNPUSHED=$(git rev-list --count origin/main..main 2>/dev/null || echo all)
  if [ "$UNPUSHED" = "0" ]; then
    echo "publish: 和上次同步比没有变化"
    exit 0
  fi
  if [ "$DRY" = "1" ]; then
    echo "publish: --dry-run，没有新改动，但有 ${UNPUSHED} 个提交还没推出去"
    exit 0
  fi
  git push -q origin main
  echo "publish: 没有新改动，把之前没推成功的 ${UNPUSHED} 个提交推上去了"
  exit 0
fi

if [ "$DRY" = "1" ]; then
  echo "publish: --dry-run，会提交这些："
  git diff --cached --stat | tail -25
  exit 0
fi

git -c user.name="DaliangAuto" -c user.email="deliangtai@gmail.com" \
    commit -q -m "firmware ${VER} (built from ${COMMIT})

整车控制板固件源码、构建脚本、以及三份对外接口文档。

先读 关于这份源码.md：这块板没有桥也不转电机，文档里的实测数字来自我们
自己那台车，是形状不是规格。

scripts/publish.sh 就是"这份树里有什么、为什么"的清单。"

git push -q origin main
echo "publish: 已推 ${VER}（${COMMIT}）到 ${REMOTE}"
