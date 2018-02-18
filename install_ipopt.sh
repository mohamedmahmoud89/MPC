# Pass the Ipopt source directory as the first argument
echo "Building Ipopt from ${srcdir}"
echo "Saving headers and libraries to ${prefix}"
# BLAS
cd ./Ipopt-3.12.7/ThirdParty/Blas
./get.Blas
mkdir -p build && cd build
../configure --prefix=/usr/local --disable-shared --with-pic
make install
# Lapack
cd ./Ipopt-3.12.7/ThirdParty/Lapack
./get.Lapack
mkdir -p build && cd build
../configure --prefix=/usr/local --disable-shared --with-pic --with-blas="/usr/local/lib/libcoinblas.a -lgfortran"
make install
# ASL
cd ./Ipopt-3.12.7/ThirdParty/ASL
./get.ASL
# MUMPS
cd ./Ipopt-3.12.7/ThirdParty/Mumps
./get.Mumps
# build everything
cd ./Ipopt-3.12.7
./configure --prefix=/usr/local coin_skip_warn_cxxflags=yes --with-blas="/usr/local/lib/libcoinblas.a -lgfortran" --with-lapack=/usr/local/lib/libcoinlapack.a
make
make test
make -j1 install