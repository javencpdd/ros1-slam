import shlex, subprocess, sys, os
B='/opt/ros1-slam/ws/build/20_localization/hdl_localization/CMakeFiles/hdl_localization_nodelet.dir'
SRC='/opt/ros1-slam/ws/src/20_localization/hdl_localization'
OUT='/tmp/asan'
os.makedirs(OUT, exist_ok=True)
def var(name):
    for line in open(f'{B}/flags.make'):
        if line.startswith(name+' ='):
            return shlex.split(line.split('=',1)[1].strip().replace('\\"','"'))
    return []
inc = var('CXX_INCLUDES')
# 手工给宏：ROS_PACKAGE_NAME 必须是带引号的字符串字面量，交给 shlex 会把引号吃掉
dfe = ['-DROSCONSOLE_BACKEND_LOG4CXX', '-DROS_BUILD_SHARED_LIBS=1',
       '-DROS_PACKAGE_NAME="hdl_localization"', '-Dhdl_localization_nodelet_EXPORTS',
       '-Dqh_QHpointer', '-DDISABLE_LIBUSB_1_0', '-DDISABLE_PCAP', '-DDISABLE_PNG']
link = open(f'{B}/link.txt').read().strip()
libs = [t for t in shlex.split(link.split('.so ',1)[1]) if not t.endswith('.o')]
base = ['-std=c++17','-fopenmp','-O1','-g','-fno-omit-frame-pointer','-fPIC','-fsanitize=address']
env = dict(os.environ, CPLUS_INCLUDE_PATH='/opt/ros/noetic/include:/opt/ros1-slam/ws/devel/include:/opt/ros1-slam/ws/.profiles/x86_sim/devel/include')
objs=[]
for rel in ('src/hdl_localization/pose_estimator.cpp','apps/hdl_localization_nodelet.cpp'):
    obj = os.path.join(OUT, os.path.basename(rel)+'.o')
    cmd = ['/usr/bin/c++']+dfe+inc+base+['-c', os.path.join(SRC,rel), '-o', obj]
    r = subprocess.run(cmd, env=env, capture_output=True, text=True)
    print(rel, 'rc=', r.returncode)
    if r.returncode: print(r.stderr[-1500:]); sys.exit(1)
    objs.append(obj)
lib = os.path.join(OUT,'libhdl_localization_nodelet.so')
cmd = ['/usr/bin/c++','-shared','-fopenmp','-fsanitize=address','-Wl,-soname,libhdl_localization_nodelet.so','-o',lib]+objs+libs
r = subprocess.run(cmd, env=env, capture_output=True, text=True)
print('link rc=', r.returncode)
if r.returncode: print(r.stderr[-2000:]); sys.exit(1)
print('built', lib, os.path.getsize(lib))
