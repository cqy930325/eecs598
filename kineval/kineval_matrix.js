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

function matrix_multiply(m1, m2) {
    var ans = [];
    if (m1[0].length != m2.length){
        return false;
    }
    var i,j,k;
    for (i=0;i<m1.length;i++) { // for each row of m1
        ans[i] = [];
        for (j=0;j<m2[0].length;j++) { // for each column of m1
            ans[i][j] = 0;
            for (k=0;k<m1[0].length;k++){
                ans[i][j]+=m1[i][k]*m2[k][j];
            }
        }
    }
    return ans;
}

function matrix_transpose(m1){
    var ans = [];
    for (i=0;i<m1.length;i++) { // for each row of m1
        ans[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            ans[i][j] = m1[j][i];
        }
    }
    return ans;
}

function generate_identity(){
    var ans = [[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]];
    return ans;
}
function generate_translation_matrix(xyz){
    var ans = [[1,0,0,xyz[0]],
                [0,1,0,xyz[1]],
                [0,0,1,xyz[2]],
                [0,0,0,1]];
    return ans;
}

function generate_rotation_matrix_X(theta){
    var cos = Math.cos(theta);
    var sin = Math.sin(theta);
    var ans = [[1,0,0,0],
                [0,cos,-sin,0],
                [0,sin,cos,0],
                [0,0,0,1]];
    return ans;

}

function generate_rotation_matrix_Y(theta){
    var cos = Math.cos(theta);
    var sin = Math.sin(theta);
    var ans = [[cos,0,sin,0],
                [0,1,0,0],
                [-sin,0,cos,0],
                [0,0,0,1]];
    return ans;

}

function generate_rotation_matrix_Z(theta){
    var cos = Math.cos(theta);
    var sin = Math.sin(theta);
    var ans = [[cos,-sin,0,0],
                [sin1,cos,0,0],
                [0,0,1,0],
                [0,0,0,1]];
    return ans;

}

function matrix_copy(mat){
    return matrix_multiply(mat,generate_identity());
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

