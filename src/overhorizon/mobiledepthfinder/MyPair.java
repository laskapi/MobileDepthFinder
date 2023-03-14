package overhorizon.mobiledepthfinder;

class MyPair<F extends Comparable<F>, S extends Comparable<S>> implements
		Comparable<MyPair<F, S>> {
	
	F first;
	S second;

	MyPair(F f, S s) {
		this.first = f;
		this.second = s;
		
	}

	public int compareTo(MyPair<F, S> another) {
		return this.first.compareTo(another.first);

	}
	
	 @Override
	    public boolean equals(Object o) {
	        if (this == o) return true;
	        if (!(o instanceof MyPair)) return false;
	      MyPair<F,S> me = (MyPair<F,S>) o;
	        return first == me.first && second == me.second;
	    }

	    @Override
	    public int hashCode() {
	        int result = first.hashCode();
	        result = 31 * result + second.hashCode();
	        return result;
	    }
	
	
	
	

}