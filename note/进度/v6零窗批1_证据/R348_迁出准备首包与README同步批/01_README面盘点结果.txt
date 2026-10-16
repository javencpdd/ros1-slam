# R348-01｜README 面盘点结果（真跑输出，时刻 2026-10-17 03:0x，零窗）

- 跑法＝`git ls-files -z` 取跟踪面 → 筛 `basename` 以 `README` 起头者（★含 `README.md`／`README.MD`／`README.rst`／`README.txt`／无后缀 `README`）→ 逐枚列其所在目录条目、检查条目名是否在该 README 正文出现（`in` 子串式、★不做语义判）→ 只报未点名者。
- ★口径一句（自纠家族追加、防账实两读）＝本 boot 早先一次口算曾报「40 枚」，那次筛的是对话内一次 `grep README` 粗数、★与是本件式不同；★该数**从未写入任何台账**（现跑 `grep -c '40 枚'` 对 `note/04_配置改动diff.md`／`note/进度/00_总览.md`／请求单／23 号＝`0/0/0/0`）⇒ 属对话侧自纠、★不涉在册原文回改；本件式跑出的**在册值＝54 枚**为当期真值，此后引用以本行为准（单位＝枚；件版本＝git HEAD `682214a` 的跟踪面 9633 枚）。
- 判读规矩＝本表只是**盘点**、★不是违宪判定：未点名 ≠ 该 README 错，尤其外部上游件（Livox SDK、Eigen、Sophus、nvbio、pybind11、hdl_*、ndt_omp、fast_gicp、livox_ros_driver2、faster_lio、kindr 等）的 README 属**他人志文**，★施工侧一律不改（在册动作＝他人志文不代改，须另开移交单）。

跟踪件总数＝9633
README 枚数＝54
其中 note/ 面＝1 ／ ws/src 面＝45 ／ scripts 面＝1 ／ 其它＝7

--- 逐枚盘点（目录内条目是否在该 README 正文被点名；只列未点名者） ---
[README.md] 目录 . → 未点名 17 枚：.cache/ .cmake/ .config/ .gazebo/ .ignition/ .nx/ .ros/ .sdformat/ .ssh/ data/ echo/ logs/ mkdir/ opt/ scripts/ tooling/ ws/
[note/进度/SCAN集成/证据/log/raw/r204_landing_candidate/README.txt] 目录 note/进度/SCAN集成/证据/log/raw/r204_landing_candidate → 未点名 1 枚：_sha256_manifest.txt
[opt/3rdparty/Livox_SDK2/3rdparty/FastCRC/README.md] 目录 opt/3rdparty/Livox_SDK2/3rdparty/FastCRC → 未点名 5 枚：FastCRC.h FastCRC_tables.h FastCRChw.cpp FastCRCsw.cpp LICENSE.md
[opt/3rdparty/Livox_SDK2/README.md] 目录 opt/3rdparty/Livox_SDK2 → 未点名 5 枚：.gitignore 3rdparty/ CHANGELOG.md CMakeLists.txt LICENSE.txt
[scripts/mapping/README.md] 目录 scripts/mapping → 未点名 2 枚：pcd_polish.py pcd_slice.py
[tooling/README.md] 目录 tooling → 未点名 1 枚：_forbid_str_exempt.tsv
[tooling/skill/senior-developer/README.md] 目录 tooling/skill/senior-developer → 未点名 8 枚：.codebuddy-plugin/ .downloaded_at MANIFEST.md MIGRATION.md SKILL.md agents/ avatars/ skills/
[ws/src/00_drivers/lite3_drivers/README.md] 目录 ws/src/00_drivers/lite3_drivers → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/00_drivers/livox_ros_driver2/README.md] 目录 ws/src/00_drivers/livox_ros_driver2 → 未点名 8 枚：.gitignore 3rdparty/ CHANGELOG.md CMakeLists.txt LICENSE.txt cmake/ package_ROS1.xml package_ROS2.xml
[ws/src/10_slam/faster_lio/README.MD] 目录 ws/src/10_slam/faster_lio → 未点名 9 枚：.clang-format .gitignore CMakeLists.txt LICENSE include/ msg/ package.xml result/ rviz_cfg/
[ws/src/10_slam/lite3_slam/README.md] 目录 ws/src/10_slam/lite3_slam → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/10_slam/pcd2grid/README.md] 目录 ws/src/10_slam/pcd2grid → 未点名 3 枚：CMakeLists.txt package.xml rviz/
[ws/src/20_localization/fast_gicp/README.md] 目录 ws/src/20_localization/fast_gicp → 未点名 9 枚：.clang-format .gitignore .gitmodules LICENSE docker/ include/ package.xml scripts/ thirdparty/
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen → 未点名 29 枚：.gitignore .gitlab/ .gitlab-ci.yml .hgeol CMakeLists.txt COPYING.APACHE COPYING.BSD COPYING.GPL COPYING.LGPL COPYING.MINPACK COPYING.MPL2 COPYING.README CTestConfig.cmake CTestCustom.cmake.in INSTALL bench/ blas/ ci/ cmake/ debug/ demos/ doc/ eigen3.pc.in failtest/ lapack/ scripts/ signature_of_eigen3_matrix_library test/ unsupported/
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench/README.txt] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench → 未点名 43 枚：BenchSparseUtil.h BenchTimer.h BenchUtil.h analyze-blocking-sizes.cpp basicbenchmark.h benchBlasGemm.cpp benchCholesky.cpp benchEigenSolver.cpp benchFFT.cpp benchGeometry.cpp benchVecAdd.cpp bench_gemm.cpp bench_move_semantics.cpp bench_norm.cpp bench_reverse.cpp bench_sum.cpp bench_unrolling benchmark-blocking-sizes.cpp benchmarkSlice.cpp benchmarkX.cpp benchmarkXcwise.cpp benchmark_suite btl/ check_cache_queries.cpp dense_solvers.cpp eig33.cpp geometry.cpp perf_monitoring/ product_threshold.cpp quat_slerp.cpp quatmul.cpp sparse_cholesky.cpp sparse_dense_product.cpp sparse_lu.cpp sparse_product.cpp sparse_randomsetter.cpp sparse_setter.cpp sparse_transpose.cpp sparse_trisolver.cpp spbench/ spmv.cpp tensors/ vdw_new.cpp
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench/btl/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench/btl → 未点名 2 枚：CMakeLists.txt COPYING
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench/tensors/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/bench/tensors → 未点名 5 枚：benchmark.h eigen_sycl_bench_contract.sh tensor_benchmarks.h tensor_benchmarks_sycl.cc tensor_contract_sycl_bench.cc
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/blas/README.txt] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/blas → 未点名 23 枚：BandTriangularSolver.h CMakeLists.txt GeneralRank1Update.h PackedSelfadjointProduct.h PackedTriangularMatrixVector.h PackedTriangularSolverVector.h Rank2Update.h common.h complex_double.cpp complex_single.cpp double.cpp f2c/ fortran/ level1_cplx_impl.h level1_impl.h level1_real_impl.h level2_cplx_impl.h level2_impl.h level2_real_impl.h level3_impl.h single.cpp testing/ xerbla.cpp
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/ci/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/ci → 未点名 4 枚：CTest2JUnit.xsl build.gitlab-ci.yml smoketests.gitlab-ci.yml test.gitlab-ci.yml
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/mandelbrot/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/mandelbrot → 未点名 3 枚：CMakeLists.txt mandelbrot.cpp mandelbrot.h
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/mix_eigen_and_c/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/mix_eigen_and_c → 未点名 1 枚：binary_library.h
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/opengl/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/demos/opengl → 未点名 11 枚：CMakeLists.txt camera.cpp camera.h gpuhelper.cpp gpuhelper.h icosphere.cpp icosphere.h quaternion_demo.cpp quaternion_demo.h trackball.cpp trackball.h
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/unsupported/Eigen/CXX11/src/Tensor/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/unsupported/Eigen/CXX11/src/Tensor → 未点名 69 枚：Tensor.h TensorArgMax.h TensorAssign.h TensorBase.h TensorBlock.h TensorBroadcasting.h TensorChipping.h TensorConcatenation.h TensorContraction.h TensorContractionBlocking.h TensorContractionCuda.h TensorContractionGpu.h TensorContractionMapper.h TensorContractionSycl.h TensorContractionThreadPool.h TensorConversion.h TensorConvolution.h TensorConvolutionSycl.h TensorCostModel.h TensorCustomOp.h TensorDevice.h TensorDeviceCuda.h TensorDeviceDefault.h TensorDeviceGpu.h TensorDeviceSycl.h TensorDeviceThreadPool.h TensorDimensionList.h TensorDimensions.h TensorEvalTo.h TensorEvaluator.h TensorExecutor.h TensorExpr.h TensorFFT.h TensorFixedSize.h TensorForcedEval.h TensorForwardDeclarations.h TensorGenerator.h TensorGlobalFunctions.h TensorGpuHipCudaDefines.h TensorGpuHipCudaUndefines.h TensorIO.h TensorImagePatch.h TensorIndexList.h TensorInflation.h TensorInitializer.h TensorIntDiv.h TensorLayoutSwap.h TensorMacros.h TensorMap.h TensorMeta.h TensorMorphing.h TensorPadding.h TensorPatch.h TensorRandom.h TensorReduction.h TensorReductionCuda.h TensorReductionGpu.h TensorReductionSycl.h TensorRef.h TensorReverse.h TensorScan.h TensorScanSycl.h TensorShuffling.h TensorStorage.h TensorStriding.h TensorTrace.h TensorTraits.h TensorUInt128.h TensorVolumePatch.h
[ws/src/20_localization/fast_gicp/thirdparty/Eigen/unsupported/README.txt] 目录 ws/src/20_localization/fast_gicp/thirdparty/Eigen/unsupported → 未点名 2 枚：CMakeLists.txt bench/
[ws/src/20_localization/fast_gicp/thirdparty/Sophus/README.rst] 目录 ws/src/20_localization/fast_gicp/thirdparty/Sophus → 未点名 18 枚：.clang-format .gitignore .travis.yml CMakeLists.txt LICENSE.txt SophusConfig.cmake.in appveyor.yml cmake_modules/ doxyfile doxyrest-config.lua examples/ make_docs.sh package.xml py/ rst-dir/ run_format.sh scripts/ test/
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio → 未点名 18 枚：BOOST.license CMakeLists.txt LICENSE cmake-local/ doxy/ examples/ nvBWT/ nvExtractReads/ nvFM-server/ nvLighter/ nvMicroAssembly/ nvSSA/ nvSetBWT/ nvbio-aln-diff/ nvbio-test/ nvmem/ sufsort-test/ sw-benchmark/
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/htslib/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/htslib → 未点名 31 枚：.travis.yml CMakeLists.txt Makefile README.nvbio bgzf.c bgzip.c config.h cram/ faidx.5 faidx.c hfile.c hfile_internal.h hfile_net.c hts.c htslib/ htslib.mk htslib_vars.mk kfunc.c knetfile.c kstring.c sam.5 sam.c synced_bcf_reader.c tabix.1 tabix.c tbx.c vcf.5 vcf.c vcf_sweep.c vcfutils.c version.h
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/htslib/README.nvbio] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/htslib → 未点名 30 枚：.travis.yml CMakeLists.txt Makefile README.md bgzf.c bgzip.c config.h cram/ faidx.5 faidx.c hfile.c hfile_internal.h hfile_net.c hts.c htslib.mk htslib_vars.mk kfunc.c knetfile.c kstring.c sam.5 sam.c synced_bcf_reader.c tabix.1 tabix.c tbx.c vcf.5 vcf.c vcf_sweep.c vcfutils.c version.h
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/libbwtsw-bin/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/libbwtsw-bin → 未点名 3 枚：bwtsw.idb bwtsw.lib bwtsw.pdb
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/libdivsufsort-lite/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/libdivsufsort-lite → 未点名 5 枚：AUTHORS VERSION divsufsort.c divsufsort.h suftest.c
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/moderngpu/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/moderngpu → 未点名 3 枚：CMakeLists.txt include/ src/
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/priority-deque/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/priority-deque → 未点名 4 枚：interval_heap.hpp main.cpp priority_deque.hpp priority_deque_verify.hpp
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/zlib-1.2.7/README] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/contrib/zlib-1.2.7 → 未点名 36 枚：CMakeLists.txt INDEX adler32.c compress.c configure.log crc32.c crc32.h deflate.h gzclose.c gzguts.h gzlib.c gzread.c gzwrite.c infback.c inffast.c inffast.h inffixed.h inflate.c inflate.h inftrees.c inftrees.h treebuild.xml trees.c trees.h uncompr.c zconf.h.cmakein zconf.h.in zlib.3 zlib.3.pdf zlib.map zlib.pc zlib.pc.cmakein zlib.pc.in zlib2ansi zutil.c zutil.h
[ws/src/20_localization/fast_gicp/thirdparty/nvbio/prototypes/psa/README.md] 目录 ws/src/20_localization/fast_gicp/thirdparty/nvbio/prototypes/psa → 未点名 4 枚：docs/ makefile src/ tools/
[ws/src/20_localization/fast_gicp/thirdparty/pybind11/README.rst] 目录 ws/src/20_localization/fast_gicp/thirdparty/pybind11 → 未点名 13 枚：.appveyor.yml .clang-format .clang-tidy .cmake-format.yaml .gitignore .pre-commit-config.yaml .readthedocs.yml CMakeLists.txt MANIFEST.in pyproject.toml setup.cfg setup.py tests/
[ws/src/20_localization/hdl_global_localization/README.md] 目录 ws/src/20_localization/hdl_global_localization → 未点名 11 枚：.clang-format .travis.yml CMakeLists.txt LICENSE config/ docker/ include/ launch/ package.xml src/ srv/
[ws/src/20_localization/hdl_localization/README.md] 目录 ws/src/20_localization/hdl_localization → 未点名 9 枚：.clang-format .travis.yml CMakeLists.txt LICENSE apps/ include/ nodelet_plugins.xml package.xml scripts/
[ws/src/20_localization/lite3_localization/README.md] 目录 ws/src/20_localization/lite3_localization → 未点名 3 枚：CMakeLists.txt config/ package.xml
[ws/src/20_localization/ndt_omp/README.md] 目录 ws/src/20_localization/ndt_omp → 未点名 8 枚：.clang-format .github/ CMakeLists.txt LICENSE apps/ docker/ include/ package.xml
[ws/src/30_perception/libraries/kindr/README.md] 目录 ws/src/30_perception/libraries/kindr → 未点名 6 枚：.gitignore cmake_uninstall.cmake.in gtesting.txt kindrConfig.cmake.in matlab/ signature_of_kindr1_library
[ws/src/30_perception/libraries/kindr_ros/README.md] 目录 ws/src/30_perception/libraries/kindr_ros → 未点名 1 枚：.gitignore
[ws/src/30_perception/lite3_perception/README.md] 目录 ws/src/30_perception/lite3_perception → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/40_navigation/lite3_navigation/README.md] 目录 ws/src/40_navigation/lite3_navigation → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/40_navigation/lite3_scan/README.md] 目录 ws/src/40_navigation/lite3_scan → 未点名 4 枚：CMakeLists.txt package.xml sink/ stub/
[ws/src/50_task/lite3_task/README.md] 目录 ws/src/50_task/lite3_task → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/60_safety/lite3_safety/README.md] 目录 ws/src/60_safety/lite3_safety → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/70_bringup/lite3_bringup/README.md] 目录 ws/src/70_bringup/lite3_bringup → 未点名 3 枚：CMakeLists.txt config/ package.xml
[ws/src/90_supervisor/lite3_supervisor/README.md] 目录 ws/src/90_supervisor/lite3_supervisor → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/95_simulation/Mid360_simulation_plugin/README.md] 目录 ws/src/95_simulation/Mid360_simulation_plugin → 未点名 1 枚：.vscode/
[ws/src/95_simulation/lite3_description/README.md] 目录 ws/src/95_simulation/lite3_description → 未点名 2 枚：CMakeLists.txt package.xml
[ws/src/95_simulation/lite3_gazebo_sim/README.md] 目录 ws/src/95_simulation/lite3_gazebo_sim → 未点名 2 枚：CMakeLists.txt package.xml

有未点名条目的 README 枚数＝51 ／ 无缺口枚数＝3

--- 施工侧分类（★四桶，逐枚归属，供追加与移交单取舍） ---

- 桶一＝★本仓自有且**有实质缺项**（本批追加句覆盖面，共 **5 枚**）：
  1. `scripts/mapping/README.md` → 缺 `pcd_polish.py`／`pcd_slice.py` 两枚脚本登记（★同批记一件权限位异常 `674`，见 `05_` 待一句件）。
  2. `tooling/README.md` → 缺 `_forbid_str_exempt.tsv`（R331 建、A36＝乙 所产）。
  3. `ws/src/40_navigation/lite3_scan/README.md` → 缺空目录 `sink/`／`stub/`（现跑文件数 0）。
  4. `ws/src/70_bringup/lite3_bringup/README.md` → 缺 `config/`（其内 `elevation_costmap.yaml`）。
  5. `ws/src/20_localization/lite3_localization/README.md` → 缺空目录 `config/`。
- 桶二＝★本仓自有、缺项仅属 **ROS 构型样板**（`CMakeLists.txt`／`package.xml`，`pcd2grid` 另含 `rviz/`）＝共 **10 枚**：`lite3_drivers`／`lite3_slam`／`pcd2grid`／`lite3_perception`／`lite3_navigation`／`lite3_task`／`lite3_safety`／`lite3_supervisor`／`lite3_description`／`lite3_gazebo_sim`（★逐行对上表本桶十行可复跑核）⇒ 施工侧判断＝★本批不为样板文件追加句（避免十处同质噪声；`package.xml` 与包名同名可推），★已在此登记为**待一句**：若主理人要「逐枚点名到底」，下批一次补齐。
- 桶三＝★外部上游件 README（他人志文）＝上表其余各枚（`opt/3rdparty/Livox_SDK2/*`、`fast_gicp/thirdparty/*` 含 Eigen 全树、`hdl_localization`／`hdl_global_localization`／`ndt_omp`／`faster_lio`／`livox_ros_driver2`／`kindr`／`kindr_ros`、`tooling/skill/senior-developer`）⇒ 零改动，★不开移交单（非本仓维护对象、逐枚点名无收益）。
- 桶四＝★两处特殊：
  1. 仓根 `README.md`＝现跑工作树**已脏**（126 行 vs HEAD 85 行，＃243 余面在册待裁）⇒ 真人明令「本件不入库、不动」，★本批零改动；其未点名的 `data/`／`scripts/`／`tooling/`／`ws/`／`opt/` 等留待 ＃243 处置同批再谈。
  2. `note/` 面**无 `README.md`**（在册门面件＝`note/00_README.md`）⇒ 本式按 basename 筛，故它只以 `note/00_README.md` 之外的形式出现、其自身未入上表（该目录条目现跑已逐条点名，缺项 0）；本批对它的追加＝★不是补缺项，而是把 **v7 定稿／`export/` 落点／`J` 后缀命名**三条新事实同步进门面。
- ★同 boot 自纠一处（登记、不消音）＝本件首版把桶二枚数写成「8 枚」并在同句括号里自记「共 10 枚」＝**同一句两个数**，违反 A32⑧（一数一单位一出处）⇒ 现跑对上表本桶十行逐枚点数＝**10**，本句为首版之误的另起句更正，首版正文已以本尾块重落（★该重落属★留底件、非台账，未涉在册原文回改）。
- 收尾核（★防止把「盘点」读成「已扫完」）＝判据仍以 `note/05_复验作业单.md` 追加三十七 为准，本件只报 **README 面**一枚式＝`有未点名条目的 README 枚数=51／无缺口枚数=3`（单位＝枚，式＝本件首段所写跑法，时刻 2026-10-17 03:06，★不得外推成任何「完成度」或「同步完毕」）。
