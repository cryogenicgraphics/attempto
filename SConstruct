#
# Copyright (C) 2015-2016 Cryogenic Graphics Ltd
#

import os
import re
import subprocess

def GetCommandOutput(command) :
    try :
        return subprocess.Popen(command, stdout = subprocess.PIPE).communicate()[0]
    except OSError :
        return None

def GetCPUInfo() :
    return GetCommandOutput(['cat', '/proc/cpuinfo'])

def GetCompilerVersion(executable) :
    version = GetCommandOutput([executable, '--version'])
    if version :
        version = re.search('[0-9]\.[0-9]\.[0-9]', version)
    return version.group(0) if version else None

def CheckForCompilerOption(context, flag) :
    context.Message('Checking for the presence of %s... ' % flag)
    cloneEnv = context.env.Clone()
    context.env.AppendUnique(CXXFLAGS = flag)
    result = context.TryCompile('int main() {}\n', '.cpp')
    if not result :
        context.env.Replace(CXXFLAGS = cloneEnv['CXXFLAGS'])
    context.Result(result)
    return result

def CheckForCpp14(context) :
    return CheckForCompilerOption(context, '-std=c++14')

def CheckForODRWarning(context) :
    return CheckForCompilerOption(context, '-Wodr')

def CheckForSuggestFinalMethods(context) :
    return CheckForCompilerOption(context, '-Wsuggest-final-methods')

def CheckForSuggestFinalTypes(context) :
    return CheckForCompilerOption(context, '-Wsuggest-final-types')

def CheckForLTOODRTypeMerging(context) :
    return CheckForCompilerOption(context, '-flto-odr-type-merging')

def CheckForARMCPU(context) :
    cpu = {'0xc07' : 'cortex-a7',
           '0xc08' : 'cortex-a8',
           '0xc09' : 'cortex-a9',
           '0xc0f' : 'cortex-a15',
           '0xd03' : 'cortex-a53'}
    context.Message('Checking for ARM... ')
    cpuinfo = GetCPUInfo()
    cpupart = [line.split(':')[1].strip() for line in cpuinfo.split('\n') if line.startswith('CPU part')]
    cpupart = sorted(tuple(set(cpupart)), reverse = True)
    cpupart = '.'.join([cpu.get(part) for part in cpupart])
    result = cpupart
    context.Result(result or 'no')
    return result

def CheckForIntelCPU(context) :
    cpu = {('6', '55', '2') : 'E3825'}
    context.Message('Checking for Intel...')
    cpuinfo = GetCPUInfo()
    cpulines = \
        [line.split(':')[1].strip()
            for line in cpuinfo.split('\n')
                for prefix in ['cpu family', 'model', 'cpu cores']
                    if line.startswith(prefix) and not line.startswith('model name')]
    cpupart = cpu.get(tuple(cpulines[:3]))
    result = cpupart
    context.Result(result or ('no' if not cpupart else 'Generic x86'))
    return result

def CheckCompiler(context, message, executable) :
    context.Message(message)
    result = GetCompilerVersion(executable)
    context.Result(result or 'no')
    return result

def CheckDefaultGCCVersion(context) :
    return CheckCompiler(context, 'Checking for GCC version...', 'gcc')

def CheckGCC5Version(context) :
    return CheckCompiler(context, 'Checking for GCC-5...', 'gcc-5')

def CheckDefaultClangVersion(context) :
    return CheckCompiler(context, 'Checking for clang version...', 'clang')

def CheckClang36Version(context) :
    return CheckCompiler(context, 'Checking for clang 3.6...', 'clang-3.6')

def CheckForReservedCFunc(context, function) :
    reservedFuncs = {'fmod'    : ('math', 'float f = 0.0f', 'f = fmod', 'f, 1.0'),
                     'memmove' : ('string', 'char c, p = 0', 'memmove', '&c, &p, 1'),
                     'memset'  : ('string', 'char c', 'memset',  '&c, 0, 1'),
                     'strtof'  : ('stdlib', 'char* p = 0, *endptr', '(void) strtof', 'p, &endptr'),}

    context.Message('Checking for C function %s()... ' % function)
    result = context.TryCompile('#include <%s.h>\nint main() {%s;%s(%s); return 0;}\n' % reservedFuncs[func], '.c')
    context.Result(result)
    return result

def AddOptions() :

    options = {'--with_clang'           : ['clang', 'build with clang'],
               '--with_fdo_generate'    : ['fdo_generate', 'build with FDO (generate step)'],
               '--with_fdo_use'         : ['fdo_use', 'build with FDO (use step)'],
               '--with_libc++'          : ['libc++', 'build with libc++'],
               '--without_debuginfo'    : ['nodebuginfo', 'build without debug information'],
               '--release'              : ['release', 'build with optimisations'],
               '--with_scanbuild'       : ['scanbuild', 'build with LLVM scan build analyser'],
               '--check'                : ['check', 'run unit tests'],
               '--clang-format'         : ['clang-format', 'run clang-format'],
               '--with_memsan'          : ['memsan', 'build with memory sanitiser'],
               '--with_ubsan'           : ['ubsan', 'build with undefined sanitiser'],
               '--with_coverage'        : ['coverage', 'build with coverage'],
               '--with_distcc'          : ['distcc', 'use distcc']}

    for option in options :
        AddOption(option, dest = options[option][0], action = 'store_true', help = options[option][1])

AddOptions()

defaultEnv = DefaultEnvironment(ENV = os.environ)

commonCXXFlags = ['-fomit-frame-pointer',
                  '-fno-math-errno',
                  '-funsigned-char',
                  '-pthread',
                  '-Werror',
                  '-Wsign-conversion',]

defaultCXXFlagsGCC = ['-D_GLIBCXX_USE_CXX11_ABI=1',
                      '-flto=jobserver',
                      '-ftree-vectorizer-verbose=1',
                      '-fuse-ld=gold',
                      '-fuse-linker-plugin',
                      '-Wall',
                      '-Wctor-dtor-privacy',
                      '-Wextra',
                      '-Wnon-virtual-dtor',
                      '-Wpedantic',
                      '-Wreorder',
                      '-Wshadow',]

defaultCXXFlagsCLANG = ['-Weverything',
                        '-Wno-c++98-compat',
                        '-Wno-c++98-compat-pedantic',
                        '-Wno-disabled-macro-expansion',
                        '-Wno-padded']

defaultEnv.AppendUnique(CPPFLAGS = commonCXXFlags)
defaultEnv.AppendUnique(CXXFLAGS = commonCXXFlags)

if GetOption('clang') :
    defaultEnv.AppendUnique(CXXFLAGS = defaultCXXFlagsCLANG)

    addedFlags = []

    if GetOption('memsan') :
        addedFlags.extend(['-fsanitize=memory', '-fno-omit-frame-pointer'])

    if GetOption('libc++') :
        addedFlags.extend(['-stdlib=libc++'])
        defaultEnv.AppendUnique(LIBS = ['c++', 'c++abi'])

    defaultEnv.AppendUnique(CXXFLAGS = addedFlags)
    defaultEnv.AppendUnique(LINKFLAGS = addedFlags)

else :
    # assume GCC

    cmdOptionToCompilerOptions = {'memsan'       : ['-fsanitize=address', '-fno-omit-frame-pointer'],
                                  'fdo_generate' : ['-fprofile-generate', '-fprofile-correction'],
                                  'fdo_use'      : ['-fprofile-use', '-fprofile-correction'],
                                  'coverage'     : ['-fprofile-arcs', '-ftest-coverage']}

    addedFlags = defaultCXXFlagsGCC
    addedFlags.extend(['--param', 'max-inline-insns-auto=100',
                       '--param', 'early-inlining-insns=200'])

    for option in cmdOptionToCompilerOptions.keys() :
        if GetOption(option) :
            addedFlags.extend(cmdOptionToCompilerOptions[option])

    defaultEnv.Append(CXXFLAGS = addedFlags)
    defaultEnv.Append(LINKFLAGS = addedFlags)

if GetOption('scanbuild') :
    defaultEnv.Replace(CXX = os.environ['CXX'])

if GetOption('distcc') :
    defaultEnv.Replace(CXX = 'distcc %s' % defaultEnv['CXX'])

nodebuginfo = GetOption('nodebuginfo')
if not nodebuginfo  :
    defaultEnv.AppendUnique(CXXFLAGS = '-ggdb' if not GetOption('clang') else '-g')

env = defaultEnv.Clone()

custom_tests = {'CheckForCpp14'                 : CheckForCpp14,
                'CheckForARMCPU'                : CheckForARMCPU,
                'CheckForReservedCFunc'         : CheckForReservedCFunc,
                'CheckForSuggestFinalMethods'   : CheckForSuggestFinalMethods,
                'CheckForSuggestFinalTypes'     : CheckForSuggestFinalTypes,
                'CheckForODRWarning'            : CheckForODRWarning,
                'CheckForLTOODRTypeMerging'     : CheckForLTOODRTypeMerging,
                'CheckForIntelCPU'              : CheckForIntelCPU,
                'CheckDefaultGCCVersion'        : CheckDefaultGCCVersion,
                'CheckGCC5Version'              : CheckGCC5Version,
                'CheckDefaultClangVersion'      : CheckDefaultClangVersion,
                'CheckClang36Version'           : CheckClang36Version,}

if not env.GetOption('clean') and not env.GetOption('clang-format') :
    conf = Configure(env, custom_tests = custom_tests)

    if not GetOption('clang') :
        gccversion = conf.CheckDefaultGCCVersion()

        env.Replace(AR = 'gcc-ar')
        env.Replace(RANLIB = 'gcc-ranlib')

        if not gccversion or not gccversion.split('.')[0] >= '5' :
            gccversion = conf.CheckGCC5Version()
            if gccversion :
                env.Replace(CPP = 'cpp-5')
                env.Replace(CC  = 'gcc-5')
                env.Replace(CXX = 'g++-5')
                env.Replace(AR = 'gcc-ar-5')
                env.Replace(RANLIB = 'gcc-ranlib-5')
    else :
        clangversion = conf.CheckDefaultClangVersion()
        if not clangversion or not clangversion.split('.')[1] >= '6' :
            clangversion = conf.CheckClang36Version()
            if clangversion :
                env.Replace(CC  = 'clang-3.6')
                env.Replace(CXX = 'clang++-3.6')
        else :
            env.Replace(CC  = 'clang')
            env.Replace(CXX = 'clang++')

    for header in ['asm/unistd.h',
                   'errno.h',
                   'fcntl.h',
                   'linux/perf_event.h',
                   'string.h',
                   'sys/ioctl.h',
                   'sys/stat.h',
                   'sys/types.h',
                   'termios.h',
                   'unistd.h' ] :
        if not conf.CheckCHeader(header) :
            Exit(1)

    for func in ['cfmakeraw',
                 'cfsetispeed',
                 'cfsetospeed',
                 'close',
                 'fcntl',
                 'isatty',
                 'open',
                 'read',
                 'strerror_r',
                 'tcflush',
                 'tcgetattr',
                 'tcsetattr',] :
        if not conf.CheckFunc(func) :
            Exit(1)

    for func in ['fmod', 'memmove', 'memset', 'strtof'] :
        if not conf.CheckForReservedCFunc(func) :
            Exit(1)

    if not conf.CheckForCpp14() :
        print('-std=c++14 is not supported by your compiler')
        Exit(1)

    conf.CheckForODRWarning()
    conf.CheckForSuggestFinalMethods()
    conf.CheckForSuggestFinalTypes()
    conf.CheckForLTOODRTypeMerging()

    for header in ['array',
                   'cassert',
                   'cmath',
                   'cstring',
                   'iostream',
                   'memory',
                   'streambuf',
                   'string',
                   'boost/test/included/unit_test.hpp'] :
        if not conf.CheckCXXHeader(header) :
            Exit(1)

    for header, lib in [('boost/program_options.hpp', 'boost_program_options')] :
        if not conf.CheckLibWithHeader(lib, header, 'c++') :
            Exit(1)

    if GetOption('ubsan') :
        conf.env.Append(CXXFLAGS = ['-fsanitize=undefined'])
        conf.env.Append(LINKFLAGS = ['-fsanitize=undefined'])

    arm = conf.CheckForARMCPU()
    if arm :
        conf.env.AppendUnique(CXXFLAGS = ['-mtune=%s' % arm])
        fpu = {'cortex-a7'            : '-mfpu=neon-vfpv4',
               'cortex-a8'            : '-mfpu=neon',
               'cortex-a9'            : '-mfpu=neon-fp16',
               'cortex-a15'           : '-mfpu=neon-vfpv4',
               'cortex-a15.cortex-a7' : '-mfpu=neon-vfpv4' }
        if arm in fpu and not GetOption('clang') :
            conf.env.AppendUnique(CXXFLAGS = ['%s' % fpu[arm]])
        elif arm in fpu :
            conf.env.AppendUnique(CXXFLAGS = ['-mfloat-abi=hard'])

    conf.CheckForIntelCPU()

    # optimisations
    if GetOption('release') :
        conf.env.AppendUnique(CPPFLAGS = ['-DNDEBUG'])
        conf.env.AppendUnique(CXXFLAGS = ['-Ofast'])

    env = conf.Finish()

if not GetOption('clang-format') :
    env['IncludeVariantSubDir'] = 'include/riot'
    env['IncludeVariantSubDirDetail'] = 'include/riot/detail'
    env['VariantDir'] = 'build/libriot'
else :
    env['IncludeVariantSubDir'] = ''
    env['IncludeVariantSubDirDetail'] = ''
    env['VariantDir'] = ''

env['ReferenceSubDir'] = 'build/test/reference'
env['ReferenceVariantSubDir'] = 'reference'

def RunUnitTest(target, source, env) :
    process = subprocess.Popen(
        [str(source[0].abspath)], stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
    stdoutdata, stderrdata = process.communicate()
    if not process.returncode :
        return open(str(target[0]), 'w').write('PASSED\n')
    else :
        print(stdoutdata, stderrdata)
        return 1

def RunUnitTestProgressString(target, source, env) :
    return 'Running tests in ' + str(source[0])

def AddUnitTest(env, target, source, *args, **kwargs) :
    env.AppendUnique(CPPDEFINES = ['-DREFERENCE_DATA_DIR=./' + env['ReferenceSubDir'] + '/'])
    program = env.Program(target, source, *args, **kwargs)

    env.Requires(program, env['ReferenceDirObj'])
    env.Clean(program, target + '.passed')

    if not env.GetOption('check') :
        return program

    unitTest = env.UnitTest(program)
    AlwaysBuild(unitTest)

    env.Alias('test', unitTest)
    env.Alias(str(program[0]), unitTest)

    return unitTest

env.Append(BUILDERS = {'UnitTest' : 
    Builder(action = env.Action(RunUnitTest, RunUnitTestProgressString), suffix = '.passed')})

def RunClangFormatProgressString(target, source, env) :
    return 'Running clang format on %s' % ' '.join((src.path for src in source))

def RunClangFormat(target, source, env) :
    if not env.GetOption('clang-format') :
        return
    process = subprocess.Popen(
        ['clang-format -i %s' % ' '.join((src.abspath for src in source))],
            stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
    process.communicate()
    assert process.returncode == 0

env.Append(BUILDERS = {'ClangFormat' :
    Builder(action =
        env.Action(RunClangFormat, RunClangFormatProgressString), suffix = '.clang_format')})

from SCons.Script.SConscript import SConsEnvironment
SConsEnvironment.AddUnitTest = AddUnitTest

def SConscript_(path, variant_dir) :
    SConscript(path, variant_dir = variant_dir, duplicate = 0, exports = 'env')

if env.GetOption('clang-format') :
  SConscript_('src/libriot/SConscript.clang_format', env['VariantDir'])
  SConscript_('test/SConscript.clang_format', env['VariantDir'])
else :
  SConscript_('src/libriot/SConscript.includes', env['VariantDir'])
  SConscript_('src/libriot/SConscript', env['VariantDir'])
  SConscript_('test/reference/SConscript', 'build/test')
  SConscript_('test/SConscript', 'build/test/nmea_reader')
