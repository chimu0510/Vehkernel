# SPDX-License-Identifier: GPL-3.0-or-later
# L1 fuzz targets for the Link decoders (ADR-0018), built by the fuzz preset
# with libFuzzer, AddressSanitizer and UndefinedBehaviorSanitizer. The root
# CMakeLists.txt instruments every source of this build; the targets here
# add libFuzzer's main().
#
# Each target runs through ctest for VK_FUZZ_SECONDS, starting from seeds
# vk_fuzz_seeds writes at build time; any sanitizer report, failed check or
# input running over 10 s fails the test and leaves the input in
# artifacts/. Longer runs are started by hand (docs/guide/testing.md).
set(VK_FUZZ_SECONDS 60 CACHE STRING "How long ctest runs each fuzz target, in seconds")

set(seed_dir ${CMAKE_CURRENT_BINARY_DIR}/seeds)
set(corpus_dir ${CMAKE_CURRENT_BINARY_DIR}/corpus)
set(artifact_dir ${CMAKE_CURRENT_BINARY_DIR}/artifacts)

add_executable(vk_fuzz_seeds make_seeds.cpp)
target_link_libraries(vk_fuzz_seeds PRIVATE vk_link vk_params vk_platform vk_warnings)
add_custom_command(OUTPUT ${seed_dir}/stamp
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${seed_dir}
  COMMAND vk_fuzz_seeds ${seed_dir}
  COMMAND ${CMAKE_COMMAND} -E touch ${seed_dir}/stamp
  DEPENDS vk_fuzz_seeds
  COMMENT "Writing the Link fuzz seeds")
add_custom_target(vk_fuzz_corpus DEPENDS ${seed_dir}/stamp)

add_custom_target(vk_fuzzers)
add_dependencies(vk_fuzzers vk_fuzz_corpus)

# vk_add_fuzzer(<name> <max input size>): builds fuzz_<name>.cpp as
# vk_fuzz_<name> and registers its bounded run as the test fuzz_<name>.
function(vk_add_fuzzer name max_len)
  add_executable(vk_fuzz_${name} fuzz_${name}.cpp)
  target_link_libraries(vk_fuzz_${name} PRIVATE vk_link vk_params vk_platform vk_warnings)
  target_link_options(vk_fuzz_${name} PRIVATE -fsanitize=fuzzer)
  add_dependencies(vk_fuzzers vk_fuzz_${name})

  # libFuzzer adds what it finds to the first directory and only reads the seeds.
  file(MAKE_DIRECTORY ${corpus_dir}/${name} ${artifact_dir})
  add_test(NAME fuzz_${name}
    COMMAND vk_fuzz_${name} -max_total_time=${VK_FUZZ_SECONDS} -max_len=${max_len} -timeout=10
      -print_final_stats=1 -artifact_prefix=${artifact_dir}/${name}-
      ${corpus_dir}/${name} ${seed_dir}/${name})
  set_tests_properties(fuzz_${name} PROPERTIES TIMEOUT 600)
endfunction()

vk_add_fuzzer(cobs 512)
vk_add_fuzzer(frame_reader 4096)
vk_add_fuzzer(session 4096)
