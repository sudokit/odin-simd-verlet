mangohud := 'false'
tracy := 'false'
release := 'false'

build:
    odin build src -out:out/a {{ if release == 'false' {'-debug'} else {'-no-bounds-check -o:speed'} }} {{ if tracy == 'true' {'-define:TRACY_ENABLE=true'} else {''} }}

run: build
    {{ if tracy == 'true' {'tracy-profiler -a 127.0.0.1 &'} else {''} }}
    {{ if mangohud == 'true' {'mangohud --dlsym'} else {''} }} ./out/a

