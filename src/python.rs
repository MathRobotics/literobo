#![allow(unsafe_op_in_unsafe_fn)]

use crate::{KinematicChain, KinematicsError};
use numpy::{PyArray2, PyReadonlyArray1, ToPyArray};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

impl From<KinematicsError> for PyErr {
    fn from(err: KinematicsError) -> Self {
        PyValueError::new_err(err.to_string())
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pyclass(name = "Robot")]
pub(crate) struct PyRobot {
    pub(crate) inner: KinematicChain,
}

#[pymethods]
impl PyRobot {
    #[staticmethod]
    fn from_urdf_file(path: &str, base_link: &str, end_link: &str) -> PyResult<Self> {
        let chain =
            KinematicChain::from_urdf_file(path, base_link.to_string(), end_link.to_string())?;
        Ok(Self { inner: chain })
    }

    #[staticmethod]
    fn from_urdf_str(urdf: &str, base_link: &str, end_link: &str) -> PyResult<Self> {
        let chain =
            KinematicChain::from_urdf_str(urdf, base_link.to_string(), end_link.to_string())?;
        Ok(Self { inner: chain })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    fn forward_kinematics<'py>(
        &self,
        py: Python<'py>,
        joints: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        if let Ok(joints_array) = joints.extract::<PyReadonlyArray1<f64>>() {
            let pose = self.inner.forward_kinematics(joints_array.as_slice()?)?;
            let matrix = pose.to_homogeneous();
            return Ok(matrix.to_pyarray_bound(py));
        }

        let joints_vec: Vec<f64> = joints.extract()?;
        let pose = self.inner.forward_kinematics(&joints_vec)?;
        let matrix = pose.to_homogeneous();
        Ok(matrix.to_pyarray_bound(py))
    }

    fn jacobian<'py>(
        &self,
        py: Python<'py>,
        joints: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        if let Ok(joints_array) = joints.extract::<PyReadonlyArray1<f64>>() {
            let jac = self.inner.jacobian(joints_array.as_slice()?)?;
            return Ok(jac.to_pyarray_bound(py));
        }

        let joints_vec: Vec<f64> = joints.extract()?;
        let jac = self.inner.jacobian(&joints_vec)?;
        Ok(jac.to_pyarray_bound(py))
    }
}

#[pyfunction(name = "from_urdf_file")]
fn py_from_urdf_file(path: &str, base_link: &str, end_link: &str) -> PyResult<PyRobot> {
    PyRobot::from_urdf_file(path, base_link, end_link)
}

#[pyfunction(name = "from_urdf_str")]
fn py_from_urdf_str(urdf: &str, base_link: &str, end_link: &str) -> PyResult<PyRobot> {
    PyRobot::from_urdf_str(urdf, base_link, end_link)
}

#[pymodule]
pub fn literobo(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRobot>()?;
    m.add("VERSION", env!("CARGO_PKG_VERSION"))?;
    m.add("BASE_LINK_KEY", "base_link")?;
    m.add("END_LINK_KEY", "end_link")?;
    m.add_function(wrap_pyfunction!(py_from_urdf_file, m)?)?;
    m.add_function(wrap_pyfunction!(py_from_urdf_str, m)?)?;

    let _ = py; // silence unused warning in non-extension builds
    Ok(())
}
