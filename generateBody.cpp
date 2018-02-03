#include"generateBody.h"



void arma_to_GL(mat &v, Mesh &body) {
	for (int i = 0; i < v.n_rows; i++)
	{
		body.vertices[i] = glm::vec4(v(i, 0), v(i, 1), v(i, 2), 1.0f);
		body.normals[i] = glm::vec3(v(i, 3), v(i, 4), v(i, 5));
	}

}

void arma_to_vector(mat &v, Vec4s &vertices, Vec3s &normals) {
	for (int i = 0; i < v.n_rows; i++)
	{
		vertices.push_back(glm::vec4(v(i, 0), v(i, 1), v(i, 2), 1.0f));
		normals.push_back(glm::vec3(v(i, 3), v(i, 4), v(i, 5)));
	}

}

SMPL::SMPL(int gender) {
	string basedir = "models";
	idd[NEUTRAL] = "neutral";
	idd[MALE] = "male";
	idd[FEMALE] = "female";
	string genderdir = idd[gender];

	f.load(basedir + "/" + genderdir + "/f.txt");
	JJ.load(basedir + "/" + genderdir + "/J.txt");
	J_regressor.load(basedir + "/" + genderdir + "/J_regressor.txt");
	kintree_table.load(basedir + "/" + genderdir + "/kintree_table.txt");
	posedirs.load(basedir + "/" + genderdir + "/posedirs.txt");
	shapedirs.load(basedir + "/" + genderdir + "/shapedirs.txt");
	v_template.load(basedir + "/" + genderdir + "/v_template.txt");
	weights.load(basedir + "/" + genderdir + "/weights.txt");
	n_template.load(basedir + "/" + genderdir + "/n_template.txt");
	kpart.load(basedir + "/" + genderdir + "/kpart.txt");
	Kinect_J_template.load(basedir + "/" + genderdir + "/Kinect_J_template.txt");
	pose_num = weights.n_cols;
	shape_num = shapedirs.n_cols;
	vert_num = v_template.n_rows;
	face_num = f.n_rows;
	Kinect_J_template.col(0) = Kinect_J_template.col(0);

	for (int i = 0; i < kintree_table.n_cols; i++) {
		id_to_col[kintree_table(1, i)] = i;
	}
	for (int i = 1; i < kintree_table.n_cols; i++) {
		parent[i] = id_to_col[kintree_table(0, i)];
	}
	mat r0;
	r0 = eye(3, 3);

	R0 = repmat(r0, 1, pose_num - 1);
}

mat SMPL::with_zeros(mat &A) {
	mat zero01;
	zero01 << 0 << 0 << 0 << 1 << endr;
	zero01.reshape(1, 4);
	mat res = join_vert(A, zero01);
	return res;
}

mat SMPL::pack(mat &A) {
	mat AA = A;
	AA.reshape(4, 1);
	mat zero43 = zeros(4, 3);
	return join_horiz(zero43, AA);
}

mat SMPL::Exp(mat &w) {
	double num = sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));
	if (num > 0.00000000000000000001) {
		mat ww = w / num;
		mat res;
		res << 0 << -ww[2] << ww[1] << endr
			<< ww[2] << 0 << -ww[0] << endr
			<< -ww[1] << ww[0] << 0 << endr;
		return eye(3, 3) + sin(num)*res + (1.0 - cos(num))*(res*res);

	}
	else {
		return eye(3, 3);
	}
}

mat SMPL::vector_to_mat(vector<mat> res) {
	uword row = res[0].n_rows;
	uword col = res[0].n_cols;
	uword size = res.size();
	mat result = zeros(size, row*col);
	int i = 0;
	mat tmp;
	for (auto it = res.begin(); it < res.end(); it++) {
		tmp = *it;
		tmp.reshape(1, row*col);
		result.row(i) = tmp;
		i++;

	}
	return result;

}

mat SMPL::compute_n(mat j1, mat j2, mat j3) {
	mat J1 = j2 - j1;
	mat J2 = j3 - j1;
	return cross(J1, J2);
}

mat SMPL::compute_t(mat x_t, mat x) {
	mat axis = -cross(x, x_t);
	axis = axis / (norm(axis) + 0.0000000000000001);
	double tmp1 = dot(x, x_t);
	axis = axis *acos(tmp1 / (norm(x_t)*norm(x) + 0.00000000001));
	return axis;

}

mat SMPL::R_to_t(mat R) {
	double tr = trace(R);
	if (tr > 3.0)
		tr = 3.0;
	else if (tr < -1.0)
		tr = -1.0;
	double theta = acos((tr - 1.0) / 2.);
	mat pp;
	pp << R(2, 1) - R(1, 2) << R(0, 2) - R(2, 0) << R(1, 0) - R(0, 1) << endr;
	pp = (abs(theta) / (2 * sin(abs(theta)) + 0.00000000001))*pp;
	return pp;
}

mat SMPL::J_to_pose(mat J) {
	J.col(0) = J.col(0);
	vector<mat> R;
	//R.push_back(eye(3, 3));
	mat n_t = compute_n(Kinect_J_template.row(0), Kinect_J_template.row(16), Kinect_J_template.row(17));
	mat n = compute_n(J.row(0), J.row(16), J.row(17));
	mat ax = compute_t(n_t, n);
	R.push_back(Exp(ax));
	for (int i = 1; i < kpart.n_rows; i++) {
		int k = int(kpart(i, 1));
		if (k == -1) {
			R.push_back(zeros(3, 3));
		}
		else {
			mat j_t = Kinect_J_template.row(kpart(i, 1)) - Kinect_J_template.row(kpart(i, 0));
			mat j1 = J.row(kpart(i, 0));
			mat j2 = J.row(kpart(i, 1));
			if (norm(j1 - zeros(1, 3)) > 0.000001&& norm(j2 - zeros(1, 3)) > 0.000001) {
				mat j = j2 - j1;
				mat axis = compute_t(j_t, j);
				R.push_back(Exp(axis));
			}
			else {
				R.push_back(zeros(3, 3));
			}
		}
	}
	mat t = zeros(kpart.n_rows, 3);
	t.row(0) = R_to_t(R[0]);
	int pp;
	mat rel_R, tmp;
	for (int i = 1; i < kpart.n_rows; i++) {
		if (norm(R[i] - zeros(3, 3)) < 0.00000001) {
			t.row(i) = zeros(1, 3);
		}
		else {
			pp = kintree_table(0, i);
			while (norm(R[pp] - zeros(3, 3)) < 0.00000001) {
				pp = kintree_table(0, pp);

			}
			tmp = trans(R[pp]) *R[i];

			t.row(i) = R_to_t(tmp);


		}
	}
	mat t_new = join_horiz(-t.col(0), t.col(1));
	t_new = join_horiz(t_new, -t.col(2));


	return t_new;
}

mat SMPL::global_rigid_transformation(mat &pose, mat &J) {

	vector<mat> results, results2;

	mat zero3 = J.row(0);
	zero3.reshape(3, 1);
	mat first = pose.row(0);
	first = Exp(first);
	first = join_horiz(first, zero3);
	first = with_zeros(first);
	results.push_back(first);
	mat tmp, tmp_pose, tmp_j;

	for (int i = 1; i < kintree_table.n_cols; i++) {
		tmp_pose = pose.row(i);
		tmp_pose = Exp(tmp_pose);
		tmp_j = J.row(i) - J.row(parent[i]);
		tmp_j.reshape(3, 1);
		tmp = join_horiz(tmp_pose, tmp_j);
		tmp = with_zeros(tmp);
		tmp = results[parent[i]] * tmp;
		results.push_back(tmp);
	}
	int i = 0;
	for (auto it = results.begin(); it < results.end(); it++) {
		tmp = zeros(4, 1);
		tmp_j = J.row(i);
		tmp_j.reshape(3, 1);
		tmp.rows(0, 2) = tmp_j;
		tmp = (*it)*tmp;
		tmp = pack(tmp);
		results2.push_back(*it - tmp);

		i++;
	}
	mat A = vector_to_mat(results2);
	return A;

}


mat SMPL::verts_core(mat &pose, mat &v, mat &J, bool want_norm) {
	mat A = global_rigid_transformation(pose, J);
	mat T = weights*A;

	mat temp_v = trans(v);
	mat one_row = ones(1, v.n_rows);

	mat rest_shape = join_vert(temp_v, one_row);
	mat sum1 = T.col(0) % trans(rest_shape.row(0))
		+ T.col(4) % trans(rest_shape.row(1))
		+ T.col(8) % trans(rest_shape.row(2))
		+ T.col(12) % trans(rest_shape.row(3));
	mat sum2 = T.col(1) % trans(rest_shape.row(0))
		+ T.col(5) % trans(rest_shape.row(1))
		+ T.col(9) % trans(rest_shape.row(2))
		+ T.col(13) % trans(rest_shape.row(3));
	mat sum3 = T.col(2) % trans(rest_shape.row(0))
		+ T.col(6) % trans(rest_shape.row(1))
		+ T.col(10) % trans(rest_shape.row(2))
		+ T.col(14) % trans(rest_shape.row(3));
	/*mat sum4 = T.col(3) % trans(rest_shape.row(0))
	+ T.col(7) % trans(rest_shape.row(1))
	+ T.col(11) % trans(rest_shape.row(2))
	+ T.col(15) % trans(rest_shape.row(3));
	*/
	mat sum = join_horiz(sum1, sum2);
	sum = join_horiz(sum, sum3);

	mat result = sum;
	if (want_norm) {
		mat zero_row = zeros(1, v.n_rows);
		mat n_ref = join_vert(trans(n_template), zero_row);
		mat n_sum1 = T.col(0) % trans(n_ref.row(0))
			+ T.col(4) % trans(n_ref.row(1))
			+ T.col(8) % trans(n_ref.row(2))
			+ T.col(12) % trans(n_ref.row(3));
		mat n_sum2 = T.col(1) % trans(n_ref.row(0))
			+ T.col(5) % trans(n_ref.row(1))
			+ T.col(9) % trans(n_ref.row(2))
			+ T.col(13) % trans(n_ref.row(3));
		mat n_sum3 = T.col(2) % trans(n_ref.row(0))
			+ T.col(6) % trans(n_ref.row(1))
			+ T.col(10) % trans(n_ref.row(2))
			+ T.col(14) % trans(n_ref.row(3));
		mat n_sum = join_horiz(n_sum1, n_sum2);
		n_sum = join_horiz(n_sum, n_sum3);
		mat mm = n_sum.col(0) % n_sum.col(0) + n_sum.col(1) % n_sum.col(1) + n_sum.col(2) % n_sum.col(2);
		mm = pow(mm, 0.5);
		n_sum = n_sum / repmat(mm, 1, 3);
		result = join_horiz(sum, n_sum);
	}
	return result;

}

void SMPL::write_to_obj(mat &v, string fname) {
	fstream obj;
	obj.open(fname, ios_base::out);

	for (int i = 0; i < v.n_rows; i++)
	{
		obj << "v ";
		for (int j = 0; j < 3; j++) {
			obj << v(i, j) << " ";

		}
		obj << endl;
	}
	if (v.n_cols == 6) {
		for (int i = 0; i < v.n_rows; i++)
		{
			obj << "vn ";
			for (int j = 3; j < 6; j++) {
				obj << v(i, j) << " ";

			}
			obj << endl;
		}
	}
	mat ff = f;
	ff++;

	for (int i = 0; i < f.n_rows; i++)
	{
		obj << "f ";
		for (int j = 0; j < f.n_cols; j++) {
			obj << ff(i, j) << " ";
		}
		obj << endl;
	}
	obj.close();
}

mat SMPL::gen_pose_model(mat &pose, bool want_norm) {
	mat J = J_regressor*v_template;
	mat R = zeros(3, (pose.n_rows - 1) * 3);
	mat tmp;
	for (int i = 0; i < pose.n_rows - 1; i++) {
		tmp = pose.row(i + 1);
		tmp = Exp(tmp);

		R.cols(3 * i, 3 * i + 2) = trans(tmp);
	}
	mat B = R - R0;
	B.reshape(1, 9 * (pose_num - 1));
	B = B*trans(posedirs);
	B.reshape(3, vert_num);
	B = trans(B);
	B = B + v_template;
	return verts_core(pose, B, J, want_norm);

}

mat SMPL::gen_full_model(mat &pose, mat &betas, bool want_norm) {
	mat S = shapedirs*betas;
	S.reshape(3, vert_num);
	S = trans(S);
	mat tmp;


	mat v = v_template + S;
	mat J = J_regressor*v;
	mat R = zeros(3, (pose.n_rows - 1) * 3);

	for (int i = 0; i < pose.n_rows - 1; i++) {
		tmp = pose.row(i + 1);
		tmp = Exp(tmp);

		R.cols(3 * i, 3 * i + 2) = trans(tmp);
	}
	mat B = R - R0;
	B.reshape(1, 9 * (pose_num - 1));
	B = B*trans(posedirs);
	B.reshape(3, vert_num);
	B = trans(B);
	v = v + B;
	return verts_core(pose, v, J, want_norm);
}


void genBody(mat JJ, Mesh &body, SMPL &obj)
{
	clock_t start = clock();
	//SMPL obj = SMPL(NEUTRAL);
	clock_t point1 = clock();
	cout << "clock: " << point1 - start << endl;
	mat pose;


	clock_t point2 = clock();

	pose = obj.J_to_pose(JJ);

	mat tmp1 = obj.J_regressor;
	mat tmp2 = obj.v_template;
	mat tmp = tmp1*tmp2;
	//pose = zeros(24, 3);
	//pose(1, 2) = 0.6;pose(16, 2) = 0.6;pose(21, 2) = 0.6;pose(10, 2) = 0.6;
	mat result = obj.gen_pose_model(pose, true);
	clock_t point3 = clock();
	cout << "clock: " << point3 - point2 << endl;
	cout << "DONE" << endl;
	//obj.write_to_obj(result, "Female_template.obj");
	arma_to_GL(result, body);
	//std::cin.get();
}


void genBodyVector(mat JJ, Vec4s &vertices, Vec3s &normals, SMPL &obj)
{
	//clock_t start = clock();
	//clock_t point1 = clock();
	//cout << "clock: " << point1 - start << endl;
	mat pose;
	////pose.row(1) = ones(1, 3)*0.3;

	clock_t point2 = clock();

	pose = obj.J_to_pose(JJ);

	mat tmp1 = obj.J_regressor;
	mat tmp2 = obj.v_template;
	mat tmp = tmp1*tmp2;
	//pose = zeros(24, 3);
	//pose(1, 2) = 0.6;pose(16, 2) = 0.6;pose(21, 2) = 0.6;pose(10, 2) = 0.6;
	mat result = obj.gen_pose_model(pose, true);
	//clock_t point3 = clock();
	//cout << "clock: " << point3 - point2 << endl;
	//cout << "DONE" << endl;
	//obj.write_to_obj(result, "Female_template.obj");
	arma_to_vector(result, vertices, normals);
}

void genFirstBody(mat pose, Vec4s &vertices, Vec3s &normals, SMPL &obj)
{
	//mat tmp1 = obj.J_regressor;
	//mat tmp2 = obj.v_template;
	//mat tmp = tmp1*tmp2;
	//pose = zeros(24, 3);
	//pose(1, 2) = 0.6;pose(16, 2) = 0.6;pose(21, 2) = 0.6;pose(10, 2) = 0.6;
	mat result = obj.gen_pose_model(pose, true);

	vertices.clear();
	normals.clear();

	//obj.write_to_obj(result, "Female_template.obj");
	arma_to_vector(result, vertices, normals);
}