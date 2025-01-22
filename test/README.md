### Function Test

#### Complie

```
nvcc -O3 -arch sm_86 -std=c++14 -Xcompiler -fPIC --use_fast_math test_sort.cu -o cuda_sort_test
```

#### Run

```
./cuda_sort_test
```

