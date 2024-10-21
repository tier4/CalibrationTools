# Troubleshooting & known issues

## Numpy errors

If the user should encounter the following error:

```text
AttributeError: module 'numpy' has no attribute 'float'.
`np.float` was a deprecated alias for the builtin `float`. To avoid this error in existing code, use `float` by itself. Doing this will not modify any behavior and is safe. If you specifically wanted the numpy scalar type, use `np.float64` here.
The aliases was originally deprecated in NumPy 1.20; for more details and guidance see the original release note at:
    https://numpy.org/devdocs/release/1.20.0-notes.html#deprecations. Did you mean: 'cfloat'?
```

It is highly due to a mismatch between the `numpy` and `transforms3d` versions in your system.
The simplest solution would be to upgrade `transforms3d` with `pip install transforms3d --upgrade`. Alternatively, the use can downgrade his version of `numpy` but doing so is more likely to incur on other issues.

At the time of writing this page, compatible versions are `numpy==1.26.4` and `transforms3d==0.4.2`
