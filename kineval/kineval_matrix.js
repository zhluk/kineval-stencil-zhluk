//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



function matrix_multiply(m1,m2) {
    // returns 2D array that is the result of m1*m2
    var r1 = m1.length;
    var c1 = m1[0].length;
    var r2 = m2.length;
    var c2 = m2[0].length;

    if (c1 !== r2){
        console.log("Matrices cannot be multiplied: Dimensional mismatch");
        return NaN;
    }

    var mat = [];
    var i,j,k;

    for (i=0;i<r1;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<c2;j++) { // for each column of m1
            mat[i][j] = 0;
            for (k=0;k<c1;k++){
                mat[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }

    return mat;
}

function matrix_transpose(m) {
    // returns 2D array that is the result of m^T
    var r1 = m.length;
    var c1 = m[0].length;

    var mat = [];
    var i,j;

    for (i=0;i<c1;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<r1;j++) { // for each column of m1
            mat[i][j] = m[j][i];
        }
    }

    return mat;
}

function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m

    var r1 = m.length;
    var c1 = m[0].length;
    var m_T = matrix_transpose(m);

    if (r1>c1){
        var m_inv = numeric.inv(matrix_multiply(m_T,m));
        return matrix_multiply(m_inv,m_T);
    } else if (r1<c1){
        var m_inv = numeric.inv(matrix_multiply(m,m_T));
        return matrix_multiply(m_T,m_inv);
    } else {
        return numeric.inv(m);
    }
}

// function matrix_invert_affine(m) {
//     // returns 2D array that is the invert affine of 4-by-4 matrix m

// }

function vector_normalize(v) {
    // returns normalized vector for v
    var len = v.length;
    var i;

    var sum = 0;
    for (i=0;i<len;i++){
        sum += v[i]*v[i];
    }

    var mag = Math.sqrt(sum);
    var v_norm = [];

    for (i=0;i<len;i++){
        v_norm[i] = v[i] / mag;
    }

    return v_norm;
}

function vector_cross(a,b) {
    // return cross product of vector a and b with both has 3 dimensions

    // if (a.length !== 3 || b.length !== 3) {
    //     console.log("Error: Input vectors must be 3-dimensional");
    //     return NaN;
    // }

    var cross =  [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
      ];

    // var a_skew = [[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]];
    // var cross = matrix_multiply(a_skew,b);

    return cross;
}

function generate_identity() {
    // returns 4-by-4 2D array of identity matrix

    var eye = [];
    var i;
    for (i = 0; i < 4; i++) {
        eye[i] = [0,0,0,0];
        eye[i][i] = 1;
    }
    
    return eye;
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array

    var D = generate_identity();
    D[0][3] = tx;
    D[1][3] = ty;
    D[2][3] = tz;

    return D;
}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);

    var R_x = [
        [1,0,0,0],
        [0,cos,-sin,0],
        [0,sin,cos,0],
        [0,0,0,1]
    ];

    return R_x;
}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);

    var R_y = [
        [cos,0,sin,0],
        [0,1,0,0],
        [-sin,0,cos,0],
        [0,0,0,1]
    ];

    return R_y;
}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    var cos = Math.cos(angle);
    var sin = Math.sin(angle);

    var R_z = [
        [cos,-sin,0,0],
        [sin,cos,0,0],
        [0,0,1,0],
        [0,0,0,1]
    ];

    return R_z;
}