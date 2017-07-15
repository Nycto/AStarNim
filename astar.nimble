# Package

version       = "0.5.0"
author        = "Nycto"
description   = "Basic A-Star path finding"
license       = "MIT"
skipDirs      = @["test", ".build", "bin"]

# Deps

requires "nim >= 0.14.0"
requires "binaryheap >= 0.1.0"

exec "test -d .build/ExtraNimble || git clone https://github.com/Nycto/ExtraNimble.git .build/ExtraNimble"
include ".build/ExtraNimble/extranimble.nim"
