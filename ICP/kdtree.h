#ifndef KDTREE_H_
#define KDTREE_H_

#include <memory>
#include <limits>
#include <queue>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
union Point3D
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float data[3];
	Point3D(float x_,float y_,float z_){x=x_;y=y_;z=z_;}
};
class kdtree {
public:
	kdtree() {}
	virtual ~kdtree() {}
	void add(const Point3D *point, const int *data) {
		kdnode::ptr node = new kdnode(point, data);
		m_nodes.push_back(node);
	}
	void build() {
		if (m_nodes.empty()) {
			return;
		}
		m_root = build(m_nodes, 0);
	}
	void clear() {
		//m_root.reset();
		m_nodes.clear();
	}
	const int *nearest_recursive(const Point3D &query) const {
		if (!m_root) {
			return NULL;
		}
		best_match best(m_root, std::numeric_limits<double>::max());
		nearest(query, m_root, best);
		return best.node->data;
	}
	void knearest(const Point3D &query, size_t k, std::vector<const int*> &result) const {
		if (!m_root || k < 1) {
			return;
		}
		MaxPriorityQueue tmp;
		knearest(query, m_root, k, tmp);
		size_t size = tmp.size();
		result.resize(size);
		for (size_t i = 0; i < size; i++) {
			// Reverse order
			result[size - i - 1] = tmp.top().second->data;
			tmp.pop();
		}
	}
	const int *nearest_iterative(const Point3D &query) const {
		if (!m_root) {
			return NULL;
		}
		MinPriorityQueue pq;
		best_match best(m_root, std::numeric_limits<double>::max());
		pq.push(DistanceTuple(0, m_root));
		while (!pq.empty()) {
			const auto current = pq.top();
			if (current.first >= best.distance) {
				return best.node->data;
			}
			pq.pop();
			auto currentNode = current.second;
			const Point3D &crp=*currentNode->split;
			double d=(query.x-crp.x)*(query.x-crp.x)+(query.y-crp.y)*(query.y-crp.y)+(query.z-crp.z)*(query.z-crp.z);
			double dx=query.data[currentNode->axis]-crp.data[currentNode->axis];
			// double d = boost::geometry::comparable_distance(query, *currentNode->split); // no sqrt
			//double dx = util::subtract(query, *currentNode->split, currentNode->axis);
			if (d < best.distance) {
				best.node = currentNode;
				best.distance = d;
			}
			node_ptr near = dx <= 0 ? currentNode->left : currentNode->right;
			node_ptr far = dx <= 0 ? currentNode->right : currentNode->left;
			if (far) pq.push(DistanceTuple(dx * dx, far));
			if (near) pq.push(DistanceTuple(0, near));
		}
		return best.node->data;
	}
private:
	struct kdnode {
		typedef kdnode* ptr;
		ptr left;
		ptr right;
		int axis;
		const Point3D *split;
		const int *data;
		kdnode(const Point3D *g, const int *d) : axis(0), split(g), data(d) {}
	};
	typedef  kdnode::ptr node_ptr; // get rid of annoying typename
	typedef std::vector<node_ptr> Nodes;
	typedef std::pair<double, node_ptr> DistanceTuple;
	struct SmallestOnTop {
		bool operator()(const DistanceTuple &a, const DistanceTuple &b) const {
			return a.first > b.first;
		}
	};
	struct LargestOnTop {
		bool operator()(const DistanceTuple &a, const DistanceTuple &b) const {
			return a.first < b.first;
		}
	};
	typedef std::priority_queue<DistanceTuple, std::vector<DistanceTuple>, SmallestOnTop> MinPriorityQueue;
	typedef std::priority_queue<DistanceTuple, std::vector<DistanceTuple>, LargestOnTop> MaxPriorityQueue;
	Nodes m_nodes;
	node_ptr m_root;

	struct Sort : std::binary_function<node_ptr, node_ptr, bool> {
		Sort(std::size_t dim) : m_dimension(dim) {}
		bool operator()(const node_ptr &lhs, const node_ptr &rhs) const {
			const Point3D &l=*lhs->split,&r=*rhs->split;
			return l.data[m_dimension]-r.data[m_dimension]<0;
			//return util::subtract(*lhs->split, *rhs->split, m_dimension) < 0;
		}
		std::size_t m_dimension;
	};
	struct best_match {
		node_ptr node;
		double distance;
		best_match(const node_ptr &n, double d) : node(n), distance(d) {}
	};

	node_ptr build(Nodes &nodes, int depth) {
		if (nodes.empty()) {
			return node_ptr();
		}
		//int axis = depth % boost::geometry::dimension<Point>();
		int axis = depth % 3;
		size_t median = nodes.size() / 2;
		std::nth_element(nodes.begin(), nodes.begin() + median, nodes.end(), Sort(axis));
		node_ptr node = nodes.at(median);
		node->axis = axis;

		Nodes left(nodes.begin(), nodes.begin() + median);
		Nodes right(nodes.begin() + median + 1, nodes.end());
		node->left = build(left, depth + 1);
		node->right = build(right, depth + 1);

		return node;
	}

	static void nearest(const Point3D &query, const node_ptr &currentNode, best_match &best) {
		if (!currentNode) {
			return;
		}
		const Point3D &crp=*currentNode->split;
		double d=(query.x-crp.x)*(query.x-crp.x)+(query.y-crp.y)*(query.y-crp.y)+(query.z-crp.z)*(query.z-crp.z);
		double dx=query.data[currentNode->axis]-crp.data[currentNode->axis];

		//double d = boost::geometry::comparable_distance(query, *currentNode->split); // no sqrt
		//double dx = util::subtract(query, *currentNode->split, currentNode->axis);
		if (d < best.distance) {
			best.node = currentNode;
			best.distance = d;
		}
		node_ptr near = dx <= 0 ? currentNode->left : currentNode->right;
		node_ptr far = dx <= 0 ? currentNode->right : currentNode->left;
		nearest(query, near, best);
		if ((dx * dx) >= best.distance) {
			return;
		}
		nearest(query, far, best);
	}
	template <typename PriorityQueue>
	static void knearest(const Point3D &query, const node_ptr &currentNode, size_t k, PriorityQueue &result) {
		if (!currentNode) {
			return;
		}
		const Point3D &crp=*currentNode->split;
		double d=(query.x-crp.x)*(query.x-crp.x)+(query.y-crp.y)*(query.y-crp.y)+(query.z-crp.z)*(query.z-crp.z);
		double dx=query.data[currentNode->axis]-crp.data[currentNode->axis];

		//double d = boost::geometry::comparable_distance(query, *currentNode->split); // no sqrt
		//double dx = util::subtract(query, *currentNode->split, currentNode->axis);
		if (result.size() < k || d <= result.top().first) {
			result.push(DistanceTuple(d, currentNode));
			if (result.size() > k) {
				result.pop();
			}
		}
		node_ptr near = dx <= 0 ? currentNode->left : currentNode->right;
		node_ptr far = dx <= 0 ? currentNode->right : currentNode->left;
		knearest(query, near, k, result);
		if ((dx * dx)  >= result.top().first) {
			return;
		}
		knearest(query, far, k, result);
	}
};

//} // namespace spatial_index

#endif /* KDTREE_H_ */
