#![allow(unsafe_op_in_unsafe_fn)]

use crate::{KinematicChain, KinematicsError};
use numpy::{PyArray2, ToPyArray};
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
        joints: Vec<f64>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let pose = self.inner.forward_kinematics(&joints)?;
        let matrix = pose.to_homogeneous();
        Ok(matrix.to_pyarray_bound(py))
    }

    fn jacobian<'py>(
        &self,
        py: Python<'py>,
        joints: Vec<f64>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let jac = self.inner.jacobian(&joints)?;
        Ok(jac.to_pyarray_bound(py))
    }

    fn with_links(&self, base_link: &str, end_link: &str) -> PyResult<Self> {
        let chain = self.inner.with_links(base_link, end_link)?;
        Ok(Self { inner: chain })
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
