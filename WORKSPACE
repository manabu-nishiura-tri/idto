# SPDX-License-Identifier: MIT-0

workspace(name = "idto")

DRAKE_COMMIT = "251bd6982d48343cbd727d0a46d887461b2d13cc"
DRAKE_CHECKSUM = "72d106c3a61383170f564916cfbae0c25285cc3cb946fca4785beea8c342b8f6"

# Or choose a specific revision of Drake to use.
# DRAKE_COMMIT = "be4f658487f739ba04ec079de46f9459b719636d"
# DRAKE_CHECKSUM = "31ec8f87df3ceb6516de3c33a14c5d59ac5c003b4faf93ac526877d2e150b394"
#
# You can also use DRAKE_COMMIT to choose a Drake release; eg:
# DRAKE_COMMIT = "v0.15.0"
#
# Before changing the COMMIT, temporarily uncomment the next line so that Bazel
# displays the suggested new value for the CHECKSUM.
# DRAKE_CHECKSUM = "0" * 64

# Or to temporarily build against a local checkout of Drake, at the bash prompt
# set an environment variable before building:
#  export IDTO_LOCAL_DRAKE_PATH=/home/user/stuff/drake

# Load an environment variable.
load("//:environ.bzl", "environ_repository")
environ_repository(name = "environ", vars = ["IDTO_LOCAL_DRAKE_PATH"])
load("@environ//:environ.bzl", IDTO_LOCAL_DRAKE_PATH = "IDTO_LOCAL_DRAKE_PATH")

# This declares the `@drake` repository as an http_archive from github,
# iff IDTO_LOCAL_DRAKE_PATH is unset.  When it is set, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
http_archive(
    name = "drake" if not IDTO_LOCAL_DRAKE_PATH else "drake_ignored",
    urls = [x.format(DRAKE_COMMIT) for x in [
        "https://github.com/RobotLocomotion/drake/archive/{}.tar.gz",
    ]],
    sha256 = DRAKE_CHECKSUM,
    strip_prefix = "drake-{}".format(DRAKE_COMMIT.lstrip("v")),
)

# This declares the `@drake` repository as a local directory,
# iff IDTO_LOCAL_DRAKE_PATH is set.  When it is unset, this declares a
# `@drake_ignored` package which is never referenced, and thus is ignored.
local_repository(
    name = "drake" if IDTO_LOCAL_DRAKE_PATH else "drake_ignored",
    path = IDTO_LOCAL_DRAKE_PATH,
)
print("Using IDTO_LOCAL_DRAKE_PATH={}".format(IDTO_LOCAL_DRAKE_PATH)) if IDTO_LOCAL_DRAKE_PATH else None  # noqa

# Reference external software libraries, tools, and toolchains per Drake's
# defaults.  Some software will come from the host system (Ubuntu or macOS);
# other software will be downloaded in source or binary form from GitHub or
# other sites.
load("@drake//tools/workspace:default.bzl", "add_default_workspace")
add_default_workspace()
