# Pinned map_lib 1919b36 uses std::reduce without an explicit initial value
# for Eigen::Vector2d. Eigen's default constructor otherwise leaves that seed
# uninitialized. Apply the supported initialization policy to ALL workspace
# targets, not just the upstream regression test, without patching upstream.
add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_ZERO)
