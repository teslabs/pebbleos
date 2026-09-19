# SPDX-FileCopyrightText: 2026 Core Devices LLC
# SPDX-License-Identifier: Apache-2.0
import argparse
import plistlib
import shutil
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description="Build a signed iPhone UI test runner")
parser.add_argument("--team", required=True, help="Apple development team identifier")
parser.add_argument("--device", required=True, help="iPhone UDID from Xcode")
parser.add_argument("--output", type=Path, default=Path("build-ios-ui-control"))
args = parser.parse_args()
root = args.output.resolve()
(root / "HfpDriver.xcodeproj/xcshareddata/xcschemes").mkdir(parents=True, exist_ok=True)
for name in ("Driver.swift", "Control.swift"):
    shutil.copyfile(Path(__file__).parent / name, root / name)
objects = {}


def obj(n, isa, **kw):
    key = f"{n:024X}"
    objects[key] = {"isa": isa, **kw}
    return key


appfile = obj(
    1,
    "PBXFileReference",
    lastKnownFileType="sourcecode.swift",
    path="Driver.swift",
    sourceTree="<group>",
)
testfile = obj(
    2,
    "PBXFileReference",
    lastKnownFileType="sourcecode.swift",
    path="Control.swift",
    sourceTree="<group>",
)
appproduct = obj(
    3,
    "PBXFileReference",
    explicitFileType="wrapper.application",
    path="HfpDriver.app",
    sourceTree="BUILT_PRODUCTS_DIR",
)
testproduct = obj(
    4,
    "PBXFileReference",
    explicitFileType="wrapper.cfbundle",
    path="HfpControl.xctest",
    sourceTree="BUILT_PRODUCTS_DIR",
)
products = obj(
    5,
    "PBXGroup",
    children=[appproduct, testproduct],
    name="Products",
    sourceTree="<group>",
)
group = obj(6, "PBXGroup", children=[appfile, testfile, products], sourceTree="<group>")
common = {
    "SDKROOT": "iphoneos",
    "IPHONEOS_DEPLOYMENT_TARGET": "17.0",
    "SWIFT_VERSION": "5.0",
    "TARGETED_DEVICE_FAMILY": "1",
    "CODE_SIGN_STYLE": "Automatic",
    "DEVELOPMENT_TEAM": args.team,
    "GENERATE_INFOPLIST_FILE": "YES",
    "ALWAYS_SEARCH_USER_PATHS": "NO",
    "SUPPORTED_PLATFORMS": "iphoneos iphonesimulator",
    "PRODUCT_NAME": "$(TARGET_NAME)",
    "SWIFT_OPTIMIZATION_LEVEL": "-Onone",
}


def config(n, extra):
    cfg = obj(
        n, "XCBuildConfiguration", name="Debug", buildSettings={**common, **extra}
    )
    return obj(
        n + 1,
        "XCConfigurationList",
        buildConfigurations=[cfg],
        defaultConfigurationIsVisible="0",
        defaultConfigurationName="Debug",
    )


appcfg = config(
    10,
    {
        "PRODUCT_BUNDLE_IDENTIFIER": "com.teslabs.hfpdriver",
        "INFOPLIST_KEY_CFBundleDisplayName": "HFP Test Control",
        "INFOPLIST_KEY_UILaunchScreen_Generation": "YES",
    },
)
testcfg = config(
    12,
    {
        "PRODUCT_BUNDLE_IDENTIFIER": "com.teslabs.hfpcontrol",
        "TEST_TARGET_NAME": "HfpDriver",
    },
)
projcfg = config(14, {})


def phases(n, file):
    build = obj(n, "PBXBuildFile", fileRef=file)
    src = obj(
        n + 1,
        "PBXSourcesBuildPhase",
        buildActionMask="2147483647",
        files=[build],
        runOnlyForDeploymentPostprocessing="0",
    )
    fw = obj(
        n + 2,
        "PBXFrameworksBuildPhase",
        buildActionMask="2147483647",
        files=[],
        runOnlyForDeploymentPostprocessing="0",
    )
    return [src, fw]


app = obj(
    30,
    "PBXNativeTarget",
    buildConfigurationList=appcfg,
    buildPhases=phases(20, appfile),
    buildRules=[],
    dependencies=[],
    name="HfpDriver",
    productName="HfpDriver",
    productReference=appproduct,
    productType="com.apple.product-type.application",
)
proxy = obj(
    32,
    "PBXContainerItemProxy",
    containerPortal=f"{40:024X}",
    proxyType="1",
    remoteGlobalIDString=app,
    remoteInfo="HfpDriver",
)
dep = obj(33, "PBXTargetDependency", target=app, targetProxy=proxy)
test = obj(
    31,
    "PBXNativeTarget",
    buildConfigurationList=testcfg,
    buildPhases=phases(23, testfile),
    buildRules=[],
    dependencies=[dep],
    name="HfpControl",
    productName="HfpControl",
    productReference=testproduct,
    productType="com.apple.product-type.bundle.ui-testing",
)
proj = obj(
    40,
    "PBXProject",
    attributes={
        "LastUpgradeCheck": "2600",
        "TargetAttributes": {test: {"TestTargetID": app}},
    },
    buildConfigurationList=projcfg,
    compatibilityVersion="Xcode 14.0",
    developmentRegion="en",
    hasScannedForEncodings="0",
    knownRegions=["en", "Base"],
    mainGroup=group,
    productRefGroup=products,
    projectDirPath="",
    projectRoot="",
    targets=[app, test],
)
with open(root / "HfpDriver.xcodeproj/project.pbxproj", "wb") as f:
    plistlib.dump(
        {
            "archiveVersion": "1",
            "classes": {},
            "objectVersion": "56",
            "objects": objects,
            "rootObject": proj,
        },
        f,
    )


def ref(key, name, product):
    return f'<BuildableReference BuildableIdentifier="primary" BlueprintIdentifier="{key}" BuildableName="{product}" BlueprintName="{name}" ReferencedContainer="container:HfpDriver.xcodeproj"/>'


(
    root / "HfpDriver.xcodeproj/xcshareddata/xcschemes/HfpDriver.xcscheme"
).write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<Scheme LastUpgradeVersion="2600" version="1.3"><BuildAction parallelizeBuildables="YES" buildImplicitDependencies="YES"><BuildActionEntries><BuildActionEntry buildForTesting="YES" buildForRunning="YES" buildForProfiling="NO" buildForArchiving="NO" buildForAnalyzing="YES">{ref(app, "HfpDriver", "HfpDriver.app")}</BuildActionEntry></BuildActionEntries></BuildAction><TestAction buildConfiguration="Debug" selectedDebuggerIdentifier="Xcode.DebuggerFoundation.Debugger.LLDB" selectedLauncherIdentifier="Xcode.IDEFoundation.Launcher.LLDB" shouldUseLaunchSchemeArgsEnv="YES"><Testables><TestableReference skipped="NO">{ref(test, "HfpControl", "HfpControl.xctest")}</TestableReference></Testables></TestAction></Scheme>""")

subprocess.run(
    [
        "xcodebuild",
        "build-for-testing",
        "-project",
        str(root / "HfpDriver.xcodeproj"),
        "-scheme",
        "HfpDriver",
        "-destination",
        f"platform=iOS,id={args.device}",
        "-derivedDataPath",
        str(root / "build"),
        "-allowProvisioningUpdates",
    ],
    check=True,
)
