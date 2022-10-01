

## Mapping script `scripts/mapping_full.py`

The script generates laser DB by shooting laser beams from each welding spot and collect hitting data.
It also runs a few scoring moethods and stores them in `result_predict.csv`.


One of the implemented scoring methods is L2 distance based on the Norm vectors. 
There are 2 `Fl_Norm` entries, each with coordinates XYZ. Each `Fl_Norm` is thought to be the 2 norm vectors for the 2 surfaces that contact and create the welding seam. 

The idea is: calculate the L2 distance for norm vectors:

```python
norms_l2_distance = np.linalg.norm(
    np.array([float(a) for a in Punkt_Fl_Norm1.split(" ")])-np.array([float(b) for b in Punkt_Fl_Norm2.split(" ")])
)
```

... and use this value to find the most similar known welding spot (that has the laser pattern).

The scoring value is currently calculated by substract the `to_predict` L2 value by L2 value of each welding spot. This is not based on any mathematic practice, but just a place holder. A proper formal should be developed for this purpose.


```python
pred_score_for_methods["Fl_norms_distance"] = norms_l2_distance - to_predict_norms_l2_distance
```