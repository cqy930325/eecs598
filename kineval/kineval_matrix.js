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
    for (i=0;i<m1[0].length;i++) { // for each row of m1
        ans[i] = [];
        for (j=0;j<m1.length;j++) { // for each column of m1
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
function generate_translation_matrix1(x,y,z){
    var ans = [[1,0,0,x],
                [0,1,0,y],
                [0,0,1,z],
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
                [sin,cos,0,0],
                [0,0,1,0],
                [0,0,0,1]];
    return ans;

}

function vector_normalize(v){
    var i;
    var ans = [];
    var norm = Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    for(i = 0;i < 3;i++){
        ans[i] = v[i]/norm;
    }
    return ans;

}
function vector_cross(u,v){
    var ans = [];
    ans[0] = u[1]*v[2] - u[2]*v[1];
    ans[1] = u[2]*v[0] - u[0]*v[2];  
    ans[2] = u[0]*v[1] - u[1]*v[0]; 
    return ans; 
}

function matrix_pseudoinverse(m){
    var M = m[0].length;
    var N = m.length;
    var mp = matrix_transpose(m);
    if (N == M){
        return numeric.inv(m);
    }
    else if (N > M){
        var mm_inv = numeric.inv(matrix_multiply(mp,m));
        return matrix_multiply(mm_inv,mp);
    }
    else{
        var mm_inv = numeric.inv(matrix_multiply(m,mp));
        return matrix_multiply(mp,mm_inv);
    }
}
function matrixtoangle(R){
    var sy = Math.sqrt(R[0][0] * R[0][0] +  R[1][0] * R[1][0]);
    if (sy > 0.000001){
        var x = Math.atan2(R[2][1],R[2][2]);
        var y = Math.atan2(R[2][0],sy);
        var z = Math.atan2(R[1][0],R[0][0]);
    }else{
        var x = Math.atan2(R[1][2],R[1][1]);
        var y = Math.atan2(R[2][0],sy);
        var z = 0;
    }
    return [x,y,z]
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

