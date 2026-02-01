def test_import_package():
    """Test that the package has the expected structure."""
    import ros2_camera_latency_benchmark

    # Check that __version__ exists
    assert hasattr(ros2_camera_latency_benchmark, "__version__"), "Package should have __version__ attribute"


def test_import_core():
    """Test that the core module can be imported."""
    from ros2_camera_latency_benchmark import core

    assert core is not None
