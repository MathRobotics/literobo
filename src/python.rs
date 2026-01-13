#![allow(unsafe_op_in_unsafe_fn)]

use crate::{KinematicChain, KinematicsError};
use nalgebra::Isometry3;
use numpy::{PyArray2, PyArrayMethods, PyReadonlyArray1, PyUntypedArrayMethods, ToPyArray};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

impl From<KinematicsError> for PyErr {
    fn from(err: KinematicsError) -> Self {
        PyValueError::new_err(err.to_string())
    }
}

fn with_joints_slice<'py, T>(
    joints: &Bound<'py, PyAny>,
    f: impl FnOnce(&[f64]) -> PyResult<T>,
) -> PyResult<T> {
    if let Ok(joints_array) = joints.extract::<PyReadonlyArray1<f64>>() {
        return f(joints_array.as_slice()?);
    }

    let joints_vec: Vec<f64> = joints.extract()?;
    f(&joints_vec)
}

fn ensure_output_shape(
    out: &Bound<'_, PyArray2<f64>>,
    rows: usize,
    cols: usize,
    name: &str,
) -> PyResult<()> {
    let shape = out.shape();
    if shape.len() != 2 || shape[0] != rows || shape[1] != cols {
        return Err(PyValueError::new_err(format!(
            "{name} must have shape ({rows}, {cols}), got ({}, {})",
            shape.get(0).copied().unwrap_or(0),
            shape.get(1).copied().unwrap_or(0)
        )));
    }
    Ok(())
}

fn write_pose(out: &Bound<'_, PyArray2<f64>>, pose: &Isometry3<f64>) {
    let rotation = pose.rotation.to_rotation_matrix();
    let rot = rotation.matrix();
    let translation = pose.translation.vector;

    for row in 0..3 {
        for col in 0..3 {
            unsafe {
                *out.uget_mut([row, col]) = rot[(row, col)];
            }
        }
        unsafe {
            *out.uget_mut([row, 3]) = translation[row];
        }
    }

    for col in 0..3 {
        unsafe {
            *out.uget_mut([3, col]) = 0.0;
        }
    }
    unsafe {
        *out.uget_mut([3, 3]) = 1.0;
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
        with_joints_slice(joints, |slice| {
            let pose = py.allow_threads(|| self.inner.forward_kinematics(slice))?;
            let matrix = pose.to_homogeneous();
            Ok(matrix.to_pyarray_bound(py))
        })
    }

    fn jacobian<'py>(
        &self,
        py: Python<'py>,
        joints: &Bound<'py, PyAny>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        with_joints_slice(joints, |slice| {
            let jac = py.allow_threads(|| self.inner.jacobian(slice))?;
            Ok(jac.to_pyarray_bound(py))
        })
    }

    fn forward_kinematics_into<'py>(
        &self,
        py: Python<'py>,
        joints: &Bound<'py, PyAny>,
        out: &Bound<'py, PyArray2<f64>>,
    ) -> PyResult<()> {
        ensure_output_shape(out, 4, 4, "out")?;
        let pose = with_joints_slice(joints, |slice| {
            Ok(py.allow_threads(|| self.inner.forward_kinematics(slice))?)
        })?;
        write_pose(out, &pose);
        Ok(())
    }

    fn jacobian_into<'py>(
        &self,
        py: Python<'py>,
        joints: &Bound<'py, PyAny>,
        out: &Bound<'py, PyArray2<f64>>,
    ) -> PyResult<()> {
        ensure_output_shape(out, 6, self.inner.dof(), "out")?;
        let jac = with_joints_slice(joints, |slice| {
            Ok(py.allow_threads(|| self.inner.jacobian(slice))?)
        })?;
        for row in 0..6 {
            for col in 0..self.inner.dof() {
                unsafe {
                    *out.uget_mut([row, col]) = jac[(row, col)];
                }
            }
        }
        Ok(())
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
